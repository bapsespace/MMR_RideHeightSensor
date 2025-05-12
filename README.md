# MMR_RideHeightSensor

## Cose da fare 
- [ ] Controllare codice "RideHeightSensor"
    - Il codice in teoria è giusto ma da errore, controllare saldature schede, risandare adc e controllare continuità

- [ ] Quando l'adc funziona in I2C, porgrammare Multiplexers TMUX1309
    - *Unused logic control pins must be tied to GND or VDD to ensure the device does not consume additional current as highlighted in Implications of Slow or Floating OS Inputs. Unused signal path inputs (Sx and Dx) should be connected to GND.*
    ![Table](/MUX%20Control%20Table.png)

    - MUX 1 
        - A0, A1 --> PB5, PB4 (su STM)
        - Tutti i pin liberi AUX
    - MUX 2 
        - A2, A3 --> PB10 , PB11 (su STM)
        - pin S1B -> Left Height (Settare questi fissi per ora )
        - pin S3A -> Right Height (Settare questi fissi per ora )
        - altri pin liberi AUX


- [ ] Quando i multiplexer funzionano, programmare CAN MCP2562-E/SN
    - CAN RX TX, PA11, PA12
    - Solito modulo CAN riciclare codice da optocouple board

- [ ] Quando tutti i sensori funzionano si può collegare i sensori di distanza , e cominciare a definire la funzione di calibrazione della misura
    - Arduino MAP function

- [ ] High level coding, main Code 
    - Loop tra i vari canali dei MUX Salva dato e manda in can
