#!/usr/bin/env python3
"""validate_ioc.py - consistency checks for CubeIDE/air_data_box.ioc against Platform/stm32h7/board_config.h.

This is NOT a CubeMX loader: it cannot prove that CubeMX accepts the file. It proves what can be proven offline:
  * the file is well-formed (no duplicate keys, counters match the lists, every pin has a signal),
  * the pins/peripherals match the board owner's board_config.h (chip selects, I2C pins, SPI/I2C/TIM handles, initial CS levels),
  * the SPI prescaler keeps SCLK <= 5 MHz for every kernel clock up to 320 MHz, and the timer is a 32-bit free-running counter.
Usage:  python3 scripts/validate_ioc.py [--datasheet-text FILE]   |   python3 scripts/validate_ioc.py --selftest
"""
import os, re, sys, tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IOC = os.path.join(ROOT, 'CubeIDE', 'air_data_box.ioc')
BCFG = os.path.join(ROOT, 'Platform', 'stm32h7', 'board_config.h')
PRESCALERS = {2, 4, 8, 16, 32, 64, 128, 256}
MAX_KERNEL_HZ = 320e6          # claim: any kernel clock up to this keeps SCLK <= 5 MHz with the chosen prescaler


def parse_ioc(text):
    kv, errors = {}, []
    lines = text.splitlines()
    if not lines or not lines[0].startswith('#MicroXplorer Configuration settings'):
        errors.append('missing MicroXplorer header line')
    for n, line in enumerate(lines[1:], start=2):
        if not line.strip():
            continue
        if '=' not in line:
            errors.append(f'line {n}: not key=value: {line[:40]!r}')
            continue
        k, v = line.split('=', 1)
        if k in kv:
            errors.append(f'line {n}: duplicate key {k}')
        kv[k] = v
    return kv, errors


def parse_board_config(text):
    d = {}
    for m in re.finditer(r'#define\s+(CS_\w+?)_PORT\s+GPIO([A-K])\b', text):
        d.setdefault(m.group(1), {})['port'] = m.group(2)
    for m in re.finditer(r'#define\s+(CS_\w+?)_PIN\s+GPIO_PIN_(\d+)\b', text):
        d.setdefault(m.group(1), {})['pin'] = m.group(2)
    cs = {k: 'P%s%s' % (v['port'], v['pin']) for k, v in d.items() if 'port' in v and 'pin' in v}
    i2c = {}
    for name in ('SCL', 'SDA'):
        p = re.search(r'#define\s+ADB_I2C_%s_PORT\s+GPIO([A-K])\b' % name, text)
        n = re.search(r'#define\s+ADB_I2C_%s_PIN\s+GPIO_PIN_(\d+)\b' % name, text)
        if p and n:
            i2c[name] = 'P%s%s' % (p.group(1), n.group(1))
    handles = dict(re.findall(r'extern\s+(?:SPI|I2C|TIM)_HandleTypeDef\s+(h\w+)\s*;\s*/\*\s*(\w+)', text))
    spi_pins = {}
    for m in re.finditer(r'extern\s+SPI_HandleTypeDef\s+(hspi\d)\s*;\s*/\*[^:]*:\s*(P\w+) SCK,\s*(P\w+) MISO,\s*(P\w+) MOSI', text):
        spi_pins[m.group(1)] = (m.group(2), m.group(3), m.group(4))
    return cs, i2c, spi_pins, handles


def validate(ioc_text, bcfg_text, datasheet_text=None):
    kv, errors = parse_ioc(ioc_text)
    err = errors.append

    # --- structure
    def count(prefix):
        return sorted(int(k[len(prefix):]) for k in kv if re.fullmatch(re.escape(prefix) + r'\d+', k))
    for prefix, nb in (('Mcu.IP', 'Mcu.IPNb'), ('Mcu.Pin', 'Mcu.PinsNb'), ('Mcu.Context', 'Mcu.ContextNb')):
        idx = count(prefix)
        if nb not in kv or not kv[nb].isdigit():
            err(f'{nb} missing'); continue
        if idx != list(range(int(kv[nb]))):
            err(f'{prefix}N entries {idx} do not match {nb}={kv[nb]}')
    for k in ('Mcu.Family', 'Mcu.Name', 'Mcu.Package', 'Mcu.UserName', 'ProjectManager.ProjectName', 'ProjectManager.TargetToolchain'):
        if k not in kv:
            err(f'{k} missing')
    if kv.get('Mcu.Name') != 'STM32H755ZITx' or kv.get('Mcu.Package') != 'LQFP144':
        err('device must be STM32H755ZITx / LQFP144')
    if kv.get('Mcu.Context0') != 'CortexM7' or kv.get('Mcu.Context1') != 'CortexM4':
        err('dual-core contexts must be CortexM7 and CortexM4')

    pins = [kv[f'Mcu.Pin{i}'] for i in count('Mcu.Pin')]
    ips = [kv[f'Mcu.IP{i}'] for i in count('Mcu.IP')]
    if len(set(pins)) != len(pins):
        err('duplicate Mcu.Pin entries')
    real = [p for p in pins if not p.startswith('VP_')]
    for p in real:
        if not re.fullmatch(r'P[A-K]\d{1,2}', p):
            err(f'{p}: not a pin name')
        if f'{p}.Signal' not in kv:
            err(f'{p}: no Signal')
        if kv.get(f'{p}.PinAttribute') != 'CortexM7':
            err(f'{p}: must be assigned to CortexM7')
        listed = kv.get(f'{p}.GPIOParameters', '').split(',') if kv.get(f'{p}.GPIOParameters') else []
        for key in listed:
            if f'{p}.{key}' not in kv:
                err(f'{p}: GPIOParameters lists {key} but {p}.{key} is missing')
    for p in [p for p in pins if p.startswith('VP_')]:
        if f'{p}.Signal' not in kv or f'{p}.Mode' not in kv:
            err(f'{p}: virtual pin needs Mode and Signal')
    for k in kv:
        m = re.match(r'(P[A-K]\d{1,2})\.Signal$', k)
        if m and m.group(1) not in pins:
            err(f'{m.group(1)} has a Signal but is not in the Mcu.Pin list')
    for ip in ('SPI2', 'SPI4', 'I2C4', 'TIM2'):
        if ip not in ips:
            err(f'{ip} missing from Mcu.IP list')

    # --- against board_config.h
    cs, i2c, spi_pins, handles = parse_board_config(bcfg_text)
    sig = {p: kv.get(f'{p}.Signal') for p in real}
    for name in ('CS_BMP581', 'CS_RSC_ADC', 'CS_RSC_EE'):
        pin = cs.get(name)
        if not pin:
            err(f'board_config.h: {name} not found'); continue
        if sig.get(pin) != 'GPIO_Output':
            err(f'{name}: board_config says {pin}, .ioc has signal {sig.get(pin)!r} there (expected GPIO_Output)')
        if kv.get(f'{pin}.PinState') != 'GPIO_PIN_SET':
            err(f'{name} ({pin}): chip select must start HIGH (PinState=GPIO_PIN_SET)')
        if kv.get(f'{pin}.GPIO_Label') != name:
            err(f'{name} ({pin}): label should be {name}')
    for name, want in (('SCL', 'I2C4_SCL'), ('SDA', 'I2C4_SDA')):
        pin = i2c.get(name)
        if not pin or sig.get(pin) != want:
            err(f'I2C {name}: board_config says {pin}, .ioc signal there is {sig.get(pin)!r} (expected {want})')
    for hspi, (sck, miso, mosi) in spi_pins.items():
        num = hspi[-1]
        for pin, fn in ((sck, 'SCK'), (miso, 'MISO'), (mosi, 'MOSI')):
            if sig.get(pin) != f'SPI{num}_{fn}':
                err(f'{hspi}: board_config says {pin} is SPI{num}_{fn}, .ioc has {sig.get(pin)!r}')
    if set(spi_pins) != {'hspi2', 'hspi4'}:
        err(f'board_config.h SPI handles found: {sorted(spi_pins)} (expected hspi2 and hspi4)')
    for h in ('hspi2', 'hspi4', 'hi2c4', 'htim2'):
        if h not in handles:
            err(f'board_config.h: handle {h} not declared')
    unexpected = [p for p in real if kv.get(f'{p}.Signal') not in
                  ('SPI4_SCK', 'SPI4_MISO', 'SPI4_MOSI', 'SPI2_SCK', 'SPI2_MISO', 'SPI2_MOSI', 'I2C4_SCL', 'I2C4_SDA', 'GPIO_Output', 'GPIO_Input')]
    if unexpected:
        err(f'unexpected signals on {unexpected}')

    # --- peripheral parameters
    for s in ('SPI2', 'SPI4'):
        m = re.fullmatch(r'SPI_BAUDRATEPRESCALER_(\d+)', kv.get(f'{s}.BaudRatePrescaler', ''))
        if not m or int(m.group(1)) not in PRESCALERS:
            err(f'{s}: invalid BaudRatePrescaler'); continue
        if MAX_KERNEL_HZ / int(m.group(1)) > 5e6:
            err(f'{s}: prescaler {m.group(1)} gives SCLK > 5 MHz at a {MAX_KERNEL_HZ/1e6:.0f} MHz kernel clock')
        if kv.get(f'{s}.NSS') != 'SPI_NSS_SOFT':
            err(f'{s}: NSS must be SPI_NSS_SOFT (chip selects are GPIOs)')
        if kv.get(f'{s}.DataSize') != 'SPI_DATASIZE_8BIT':
            err(f'{s}: data size must be 8 bit (H7 default is 4 bit)')
        if kv.get(f'{s}.Mode') != 'SPI_MODE_MASTER' or kv.get(f'{s}.Direction') != 'SPI_DIRECTION_2LINES':
            err(f'{s}: must be full-duplex master')
        for key in kv.get(f'{s}.IPParameters', '').split(','):
            if key and f'{s}.{key}' not in kv:
                err(f'{s}: IPParameters lists {key} but {s}.{key} is missing')
    if kv.get('TIM2.Period') != '4294967295':
        err('TIM2: period must be 4294967295 (32-bit free-running)')
    if not kv.get('TIM2.Prescaler', '').isdigit():
        err('TIM2: prescaler missing')

    # --- optional: every signal name must exist in the datasheet text
    if datasheet_text is not None:
        for fn in sorted({s for s in sig.values() if s and not s.startswith('GPIO_')}):
            if fn not in datasheet_text:
                err(f'datasheet text does not mention {fn}')
        for p in real:
            if not re.search(r'\b%s\b' % p, datasheet_text):
                err(f'datasheet text does not mention pin {p}')
    return errors


def selftest(ioc_text, bcfg_text):
    """The validator must reject deliberately broken variants (otherwise a pass means nothing)."""
    cases = [
        ('chip select moved to another pin', lambda t: t.replace('PE11.Signal=GPIO_Output', 'PE11.Signal=GPIO_Input')),
        ('chip select starts LOW', lambda t: t.replace('PE11.PinState=GPIO_PIN_SET\n', '')),
        ('SCL signal on wrong pin', lambda t: t.replace('PD12.Signal=I2C4_SCL', 'PD12.Signal=I2C4_SDA')),
        ('SPI2 MISO missing', lambda t: t.replace('PB14.Signal=SPI2_MISO', 'PB14.Signal=SPI2_SCK')),
        ('IP counter wrong', lambda t: t.replace('Mcu.IPNb=11', 'Mcu.IPNb=12')),
        ('pin counter wrong', lambda t: t.replace('Mcu.PinsNb=16', 'Mcu.PinsNb=15')),
        ('duplicate key', lambda t: t + 'Mcu.Family=STM32H7\n'),
        ('SPI prescaler too small (SCLK > 5 MHz)', lambda t: t.replace('SPI4.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_64', 'SPI4.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_8')),
        ('SPI data size left at 4 bit', lambda t: t.replace('SPI2.DataSize=SPI_DATASIZE_8BIT', 'SPI2.DataSize=SPI_DATASIZE_4BIT')),
        ('hardware NSS', lambda t: t.replace('SPI4.NSS=SPI_NSS_SOFT', 'SPI4.NSS=SPI_NSS_HARD_OUTPUT')),
        ('timer not 32-bit period', lambda t: t.replace('TIM2.Period=4294967295', 'TIM2.Period=65535')),
        ('pin not assigned to CM7', lambda t: t.replace('PE12.PinAttribute=CortexM7', 'PE12.PinAttribute=CortexM4')),
        ('Signal on pin missing from Mcu.Pin list', lambda t: t + 'PA5.Signal=GPIO_Output\n'),
        ('wrong device', lambda t: t.replace('Mcu.Name=STM32H755ZITx', 'Mcu.Name=STM32H743ZITx')),
        ('missing header', lambda t: t.split('\n', 1)[1]),
    ]
    bad = 0
    for name, mutate in cases:
        if not validate(mutate(ioc_text), bcfg_text):
            print(f'  SELFTEST FAIL: validator accepted a broken file ({name})'); bad += 1
        else:
            print(f'  [rejected] {name}')
    # a board_config.h that disagrees with the .ioc must also be caught
    if not validate(ioc_text, bcfg_text.replace('GPIO_PIN_11', 'GPIO_PIN_10', 1)):
        print('  SELFTEST FAIL: validator accepted a board_config.h that disagrees with the .ioc'); bad += 1
    else:
        print('  [rejected] board_config.h disagrees with the .ioc')
    return bad


def main(argv):
    ioc_text = open(IOC, encoding='utf-8').read()
    bcfg_text = open(BCFG, encoding='utf-8').read()
    if '--selftest' in argv:
        base = validate(ioc_text, bcfg_text)
        if base:
            print('SELFTEST: the real file must validate first:', base); return 1
        bad = selftest(ioc_text, bcfg_text)
        print('validator self-test:', 'FAILED' if bad else 'all broken variants rejected')
        return 1 if bad else 0
    ds = None
    if '--datasheet-text' in argv:
        ds = open(argv[argv.index('--datasheet-text') + 1], encoding='utf-8', errors='ignore').read()
    errors = validate(ioc_text, bcfg_text, ds)
    for e in errors:
        print('IOC ERROR:', e)
    print('ioc validation:', 'FAILED (%d problems)' % len(errors) if errors else 'OK (consistent with board_config.h)')
    return 1 if errors else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
