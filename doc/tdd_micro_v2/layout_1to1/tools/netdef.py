# Net assignment: copper-verified where possible, otherwise the reconstructed schematic + firmware
import fp
# MCU pin -> functional net name (F407VG_DISCO firmware + board silkscreen)
MCU={'PE9':'PWM','PE11':'DIR','PA0':'ENC_A','PA1':'ENC_B','PA2':'ENC_I','PA4':'PA4_SPI1_CS','PA5':'PA5_SPI1_SCK',
     'PA6':'SPI1_MISO','PA7':'SPI1_MOSI','PB12':'SPI2_CS','PB13':'SPI2_SCK','PB14':'SPI2_MISO','PB15':'SPI2_MOSI',
     'PB1':'GAS','PB0':'BRAKE','PC4':'CLUTCH','PC5':'Z_SENSOR','PC2':'SHIFT_X','PC1':'SHIFT_Y',
     'PD12':'LED_G','PD14':'LED_R','PD15':'LED_B','PA13':'SWDIO','PA14':'SWCLK',
     'PE0':'BTN1','PE1':'BTN2','PE2':'BTN3','PE3':'BTN4','PE4':'BTN5','PE5':'BTN6','PC6':'BTN7','PC7':'BTN8',
     'PC8':'BTN9','PC9':'BTN10','PC10':'BTN11','PC11':'BTN12','PC12':'BTN13','PC13':'BTN14',
     'GND':'GND','3V3':'+3V3','5V':'+5V','BOOT':'BOOT'}
def assign():
    N={}
    for f,p in fp.all_pads():
        r,n,nm=f['ref'],p['num'],p.get('name','')
        k=(r,n)
        if n=='MH' and r!='J2': N[k]=''; continue
        if r=='J1': N[k]=MCU.get(nm,'')
        elif r=='J6': N[k]={'GND':'GND','AREF':'+3V3','SWDIO':'SWDIO','SWCLK':'SWCLK'}[nm]
        elif r=='J7': N[k]='GND' if nm=='GND' else 'BTN'+nm[1:]
        elif r=='J5': N[k]={'GND':'GND','3V3':'+3V3','Z':'Z_SENSOR'}[nm]
        elif r=='J4': N[k]={'SCK':'PA5_SPI1_SCK','MISO':'SPI1_MISO','CS':'PA4_SPI1_CS','X':'SHIFT_X','Y':'SHIFT_Y','3V3':'+3V3','GND':'GND','NC':''}[nm]
        elif r=='J3': N[k]={'3V3':'+3V3','GAS':'GAS','BRAKE':'BRAKE','CLUTCH':'CLUTCH','GND':'GND','NC':''}[nm]
        elif r=='J2':
            N[k]={'6':'SON_ESTOP','7':'DIR','9':'+5V','10':'GND','13':'GND','15':'ENC_I','18':'ENC_A','20':'ENC_B','25':'TORQUE','MH':'SHIELD'}.get(n,'')
        elif r=='J8': N[k]={'1':'SON_ESTOP','2':'GND'}[n]
        elif r in ('D1','D2','D3'):
            col={'D1':'G','D2':'B','D3':'R'}[r]
            N[k]=f'LED_{col}' if n=='1' else f'LED_{col}_K'
        elif r in ('R1','R2','R3'):
            col={'R1':'G','R2':'B','R3':'R'}[r]
            N[k]='GND' if n=='1' else f'LED_{col}_K'
        elif r=='C1': N[k]='+3V3' if n=='1' else 'GND'
        elif r=='C2': N[k]='+3V3' if n=='1' else 'GND'
        elif r=='C3': N[k]='DAC_REF' if n=='1' else 'GND'
        elif r=='U1':
            N[k]={'VCC':'+3V3','IOVCC':'+3V3','PD':'+3V3','IDLSEL':'IDLSEL','REFSEL':'REFSEL','INA':'DAC_INA','INB':'DAC_INB',
                  'VOUTA':'DAC_OUTA','VOUTB':'DAC_OUTB','REF':'DAC_REF','GND':'GND'}[nm]
        elif r=='JP1': N[k]='DAC_SEL' if n=='1' else 'DAC_OUTA'
        elif r=='JP2': N[k]='DAC_SEL' if n=='1' else 'DAC_OUTB'
        elif r=='JP3': N[k]='DAC_INA' if n=='1' else 'DIR'
        elif r=='JP4': N[k]='DAC_INB' if n=='1' else 'PWM'
        elif r=='JP5': N[k]='PA5_SPI1_SCK' if n=='1' else 'TORQUE'
        elif r=='JP6': N[k]='PA4_SPI1_CS' if n=='1' else 'TORQUE'
        elif r=='JP7': N[k]='DAC_SEL' if n=='1' else 'TORQUE'
        elif r=='JP8': N[k]='SHIELD' if n=='1' else 'GND'
        else: N[k]=''
    return N
