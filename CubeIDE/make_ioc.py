#!/usr/bin/env python3
"""Generates CubeIDE/air_data_box.ioc.

Structure copied from a real CubeMX 6.11.1 dual-core STM32H755 project file (same key families: Mcu.*, CortexM7.IPs,
NVIC1/NVIC2, <pin>.Signal/Mode/PinAttribute, ProjectManager.*). The peripheral content comes from the board document
(13.11.2026) and Platform/stm32h7/board_config.h. NEVER LOADED IN CUBEMX (no CubeMX available when written): see docs/CUBEIDE_SETUP.md.
Run:  python3 CubeIDE/make_ioc.py   then   python3 scripts/validate_ioc.py
"""
import os
pins = {}
def pin(name, signal, mode=None, label=None, state=None, speed=None):
    d, params = {}, []
    if label: params.append('GPIO_Label'); d['GPIO_Label'] = label
    if speed: params.append('GPIO_Speed'); d['GPIO_Speed'] = speed
    if state: params.append('PinState'); d['PinState'] = state
    params.append('PinAttribute'); d['PinAttribute'] = 'CortexM7'
    d['GPIOParameters'] = ','.join(params); d['Signal'] = signal
    if mode: d['Mode'] = mode
    pins[name] = d
HI = 'GPIO_SPEED_FREQ_HIGH'
pin('PE12', 'SPI4_SCK',  'Full_Duplex_Master', speed=HI)   # BMP581 on SPI4
pin('PE13', 'SPI4_MISO', 'Full_Duplex_Master')
pin('PE14', 'SPI4_MOSI', 'Full_Duplex_Master', speed=HI)
pin('PB13', 'SPI2_SCK',  'Full_Duplex_Master', speed=HI)   # RSC on SPI2
pin('PB14', 'SPI2_MISO', 'Full_Duplex_Master')
pin('PB15', 'SPI2_MOSI', 'Full_Duplex_Master', speed=HI)
pin('PD12', 'I2C4_SCL', 'I2C')                             # MS5611 on I2C4
pin('PD13', 'I2C4_SDA', 'I2C')
pin('PE11', 'GPIO_Output', label='CS_BMP581',  state='GPIO_PIN_SET')   # chip selects start HIGH
pin('PB12', 'GPIO_Output', label='CS_RSC_ADC', state='GPIO_PIN_SET')
pin('PD11', 'GPIO_Output', label='CS_RSC_EE',  state='GPIO_PIN_SET')
pin('PD15', 'GPIO_Input', label='BMP581_INT')              # optional, unused by the firmware
pin('PD8',  'GPIO_Input', label='RSC_DRDY')                # optional, unused by the firmware
vps = ['VP_SYS_M4_VS_Systick', 'VP_SYS_VS_Systick', 'VP_TIM2_VS_ClockSourceINT']
ips = ['CORTEX_M4', 'CORTEX_M7', 'I2C4', 'NVIC1', 'NVIC2', 'RCC', 'SPI2', 'SPI4', 'SYS', 'SYS_M4', 'TIM2']
mcu_pins = ['PB12', 'PB13', 'PB14', 'PB15', 'PD8', 'PD11', 'PD12', 'PD13', 'PD15', 'PE11', 'PE12', 'PE13', 'PE14'] + vps
L = []
def add(k, v=''): L.append(f'{k}={v}')
for k in ('CAD.formats', 'CAD.pinconfig', 'CAD.provider'): add(k)
add('CORTEX_M7.IPParameters', 'default_mode_Activation'); add('CORTEX_M7.default_mode_Activation', '1')
add('CortexM4.IPs', 'RCC,DMA,BDMA,MDMA,NVIC2\\:I,DEBUG,PWR,SYS_M4\\:I,CORTEX_M4\\:I,GPIO,VREFBUF,IWDG2\\:I,WWDG2\\:I')
add('CortexM7.IPs', 'RCC\\:I,DMA\\:I,BDMA\\:I,MDMA\\:I,NVIC1\\:I,SYS\\:I,CORTEX_M7\\:I,DEBUG\\:I,PWR\\:I,GPIO\\:I,VREFBUF\\:I,IWDG1\\:I,WWDG1\\:I,I2C4\\:I,SPI2\\:I,SPI4\\:I,TIM2\\:I')
add('File.Version', '6'); add('GPIO.groupedBy', 'Group By Peripherals'); add('KeepUserPlacement', 'false')
add('Mcu.CPN', 'STM32H755ZIT6'); add('Mcu.Context0', 'CortexM7'); add('Mcu.Context1', 'CortexM4'); add('Mcu.ContextNb', '2'); add('Mcu.Family', 'STM32H7')
for i, n in enumerate(ips): add(f'Mcu.IP{i}', n)
add('Mcu.IPNb', str(len(ips))); add('Mcu.Name', 'STM32H755ZITx'); add('Mcu.Package', 'LQFP144')
for i, n in enumerate(mcu_pins): add(f'Mcu.Pin{i}', n)
add('Mcu.PinsNb', str(len(mcu_pins))); add('Mcu.ThirdPartyNb', '0'); add('Mcu.UserConstants'); add('Mcu.UserName', 'STM32H755ZITx')
add('MxCube.Version', '6.11.1'); add('MxDb.Version', 'DB.6.0.111')
fault = 'true\\:0\\:0\\:false\\:false\\:true\\:false\\:false\\:false'
for core in ('NVIC1', 'NVIC2'):
    for irq in ('BusFault', 'DebugMonitor', 'HardFault', 'MemoryManagement', 'NonMaskableInt', 'PendSV', 'SVCall', 'UsageFault'):
        add(f'{core}.{irq}_IRQn', fault)
    add(f'{core}.ForceEnableDMAVector', 'true'); add(f'{core}.PriorityGroup', 'NVIC_PRIORITYGROUP_4')
    add(f'{core}.SysTick_IRQn', 'true\\:15\\:0\\:false\\:false\\:true\\:false\\:true\\:false')
for name, d in pins.items():
    for k, v in d.items(): add(f'{name}.{k}', v)
add('PinOutPanel.RotationAngle', '0')
pm = {'AskForMigrate': 'true', 'BackupPrevious': 'false', 'BootMode': 'boot0', 'CompilerOptimize': '6', 'ComputerToolchain': 'false',
      'CoupleFile': 'false', 'CustomerFirmwarePackage': '', 'DefaultFWLocation': 'true', 'DeletePrevious': 'true', 'DeviceId': 'STM32H755ZITx',
      'FreePins': 'false', 'HalAssertFull': 'false', 'HeapSize': 'M4-0x200,M7-0x400', 'KeepUserCode': 'true', 'LastFirmware': 'true', 'LibraryCopy': '1',
      'MainLocation': 'Core/Src', 'NoMain': 'false', 'PreviousToolchain': 'STM32CubeIDE', 'ProjectBuild': 'false', 'ProjectFileName': 'air_data_box.ioc',
      'ProjectName': 'air_data_box', 'ProjectStructure': 'M7\\:CortexM7 Project\\:true;M4\\:CortexM4 Project\\:true;', 'RegisterCallBack': '',
      'StackSize': 'M4-0x400,M7-0x2000', 'TargetToolchain': 'STM32CubeIDE', 'ToolChainLocation': '', 'UAScriptAfterPath': '', 'UAScriptBeforePath': '', 'UnderRoot': 'true',
      'functionlistsort': '1-SystemClock_Config-RCC-false-HAL-false-CortexM7,2-MX_GPIO_Init-GPIO-false-HAL-true-CortexM7,3-MX_I2C4_Init-I2C4-false-HAL-true-CortexM7,'
                          '4-MX_SPI2_Init-SPI2-false-HAL-true-CortexM7,5-MX_SPI4_Init-SPI4-false-HAL-true-CortexM7,6-MX_TIM2_Init-TIM2-false-HAL-true-CortexM7,'
                          '0-MX_CORTEX_M7_Init-CORTEX_M7-false-HAL-true-CortexM7,0-MX_CORTEX_M4_Init-CORTEX_M4-false-HAL-true-CortexM4'}
for k, v in pm.items(): add(f'ProjectManager.{k}', v)
# SPI: prescaler /64 keeps SCLK <= 5 MHz for ANY kernel clock up to 320 MHz, so it needs no recalculation when the clock tree arrives.
for s in ('SPI2', 'SPI4'):
    add(f'{s}.BaudRatePrescaler', 'SPI_BAUDRATEPRESCALER_64'); add(f'{s}.DataSize', 'SPI_DATASIZE_8BIT'); add(f'{s}.Direction', 'SPI_DIRECTION_2LINES')
    add(f'{s}.IPParameters', 'VirtualType,Mode,Direction,DataSize,NSS,BaudRatePrescaler'); add(f'{s}.Mode', 'SPI_MODE_MASTER'); add(f'{s}.NSS', 'SPI_NSS_SOFT'); add(f'{s}.VirtualType', 'VM_MASTER')
# TIM2: free-running 32-bit counter. Prescaler 63 = 1 MHz ONLY for a 64 MHz timer clock (CubeMX default HSI tree). Recompute with the real clock;
# the firmware refuses to start if the time base is not 1 MHz (adb_time_base_ok).
add('TIM2.IPParameters', 'Prescaler,Period'); add('TIM2.Period', '4294967295'); add('TIM2.Prescaler', '63')
for v, m in (('VP_SYS_M4_VS_Systick', 'SysTick'), ('VP_SYS_VS_Systick', 'SysTick'), ('VP_TIM2_VS_ClockSourceINT', 'Internal')):
    add(f'{v}.Mode', m); add(f'{v}.Signal', v[3:])
body = sorted(L, key=lambda s: s.split('=', 1)[0])        # CubeMX writes keys in sorted order
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'air_data_box.ioc')
open(out, 'w', newline='\n').write('#MicroXplorer Configuration settings - do not modify\n' + '\n'.join(body) + '\n')
print('wrote', out, len(body) + 1, 'lines')
