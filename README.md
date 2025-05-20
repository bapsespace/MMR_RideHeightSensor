# MMR_RideHeightSensor

## Note sul PCB
- [x] PT6 NON COLLEGATO, errore design PCB, (_non è indispinsabile basta ricordarsi di non usarlo_)
- [ ] Da risaldare OPAMP e i 2 FUsibili per laser distance sensors

## Note sul Codice
- [ ] **IMPLEMENTARE CAN**
- [ ] aggiungere funzione conversione, adc_raw -> adce_voltage  (-32k : +32K [signed int 16bit]  -> -2.048V : +2.048V [float])
- [ ] Aggiungere, funzione calibrazione specifica **solo** per Sensori Laser Distanza

## Note Varie

- [x] L'ADC funziona con il codice: "RideHeighSensorV3_ADC_OK"

- [x] Quando l'adc funziona in I2C, porgrammare Multiplexers TMUX1309
    - *Unused logic control pins must be tied to GND or VDD to ensure the device does not consume additional current as highlighted in Implications of Slow or Floating OS Inputs. Unused signal path inputs (Sx and Dx) should be connected to GND.*
    ![Table](/MUX%20Control%20Table.png)

```
    MUX1				PB4	PB5
	(DA --> AIN0)			A1	A0
		[S0A --> ADC1]		0	0	ok	FUNZIONA
		[S1A --> ]		0	1	ok	-
		[S2A --> PT1]		1	0	ok	FUNZIONA
		[S3A --> ADC2]		1	1	ok	FUNZIONA

	(DB --> AIN1)	
		[S0B --> PT2]		0	0	ok	FUNZIONA
		[S1B --> PT8]		0	1	ok	FUNZIONA
		[S2B --> PT4]		1	0	ok	FUNZIONA
		[S3B --> PT6]		1	1	ok	-DA CONTROLLARE


    MUX2				PB10	PB11
	(DA --> AIN2)			A1	A0
		[S0A --> PT3]		0	0	ok	FUNZIONA
		[S1A --> PT5]		0	1	ok	FUNZIONA
		[S2A --> PT7]		1	0	ok	FUNZIONA
		[S3A --> Height_Right]	1	1	ok	FUNZIONA

	(DB --> AIN3)			
		[S0B --> ]		0	0	ok	-
		[S1B --> Height_Left]	0	1	ok	FUNZIONA
		[S2B --> ]		1	0	ok	-
		[S3B --> ]		1	1	ok	-
```

- [ ] Quando i multiplexer funzionano, programmare CAN MCP2562-E/SN
    - CAN RX TX, PA11, PA12
    - Solito modulo CAN riciclare codice da optocouple board

- [ ] Quando tutti i sensori funzionano si può collegare i sensori di distanza , e cominciare a definire la funzione di calibrazione della misura
    - Arduino MAP function

- [ ] High level coding, main Code 
    - Loop tra i vari canali dei MUX Salva dato e manda in can
