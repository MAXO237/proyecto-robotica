# proyecto-robotica
## Ideas
Para la raspberry, se utiliza la biblioteca smbus2. Escpecíficamente se utiliza el método
```python
-- para enviar datos:
write_i2c_block_data(i2c_addr, register, data, force=None)
```
La documentación de este método está [acá](https://smbus2.readthedocs.io/en/latest/)

Para el arduino, se utiliza la biblioteca #include <Wire.h>. Específicamente se utiliza el método
```cpp
Wire.begin(SLAVE_ADDRESS);
Wire.onReceive(receiveEvent);
Wire.onRequest(requestEvent);
Wire.available()
//Para recibir los datos:
Wire.read()
```
## Ligas Importantes
