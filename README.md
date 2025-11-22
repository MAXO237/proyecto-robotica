# proyecto-robotica
## Ideas
Para la raspberry, se utiliza la biblioteca smbus2. Escpecíficamente se utiliza el método
```python
# para enviar datos:
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
- Quetzalcoatl. [acá](https://quetzalcoatl.fciencias.unam.mx/taller-de-robotica/index.php/recursos-para-organizadores/raspberrypi-os-para-el-taller/)
- Repositorio Base, que controla con un joystick a un robot en rviz. [acá](https://github.com/veroarriola/viz_package_cpp/tree/main/src)
- Repositorio de paquito zero, utiliza i2c. [acá](https://github.com/veroarriola/viz_package_cpp/tree/main/src)
- Repositorio EscuelaPC>Paquito. Es el repo que tiene la raspberry de paquito. [acá](https://github.com/taller-de-robotica/Escuela-pc/tree/main/Paquito)
- Repositorio taller-de-robotica>Paquito>i2c. Es con el que probamos el protocolo i2c. [acá](https://github.com/taller-de-robotica/paquito/tree/main/i2c)

## Archivos con los que probamos el protocolo i2c:
- Archivo de python i2ctest.py, va en la raspberry de paquito. taller-de-robotica/paquito/i2c
	[https://github.com/taller-de-robotica/paquito/tree/main/i2c](https://github.com/taller-de-robotica/paquito/tree/main/i2c)
- Archivo sketch_i2c_test.ino que cargamos con la laptop al arduino de paquito. taller-de-robotica/Escuela-pc/Paquito
	[https://github.com/taller-de-robotica/Escuela-pc/tree/main/Paquito](https://github.com/taller-de-robotica/Escuela-pc/tree/main/Paquito)

## Ligas de Aprendizaje
- [https://core-electronics.com.au/courses/arduino-workshop-for-beginners/](https://core-electronics.com.au/courses/arduino-workshop-for-beginners/)

## Otros
- Código fuente de la función `write_i2c_block_data`: [https://github.com/kplindegaard/smbus2/blob/master/smbus2/smbus2.py](https://github.com/kplindegaard/smbus2/blob/master/smbus2/smbus2.py)
```python
def write_i2c_block_data(self, i2c_addr, register, data, force=None):
        """
        Write a block of byte data to a given register.

        :param i2c_addr: i2c address
        :type i2c_addr: int
        :param register: Start register
        :type register: int
        :param data: List of bytes
        :type data: list
        :param force: Use slave address even when driver is already using it.
        :type force: bool
        :rtype: None
        """
        length = len(data)
        if length > I2C_SMBUS_BLOCK_MAX:
            raise ValueError("Data length cannot exceed %d bytes" % I2C_SMBUS_BLOCK_MAX)
        self._set_address(i2c_addr, force=force)
        msg = i2c_smbus_ioctl_data.create(
            read_write=I2C_SMBUS_WRITE, command=register, size=I2C_SMBUS_I2C_BLOCK_DATA
        )
        msg.data.contents.block[0] = length
        msg.data.contents.block[1:length + 1] = data
        ioctl(self.fd, I2C_SMBUS, msg)
```
- Ejemplo para escribir un bloque de datos con smbus2:
  "It is possible to write 32 bytes at the time, but I have found that error-prone. Write less and add a delay in between if you run into trouble."
  ```python
  from smbus2 import SMBus
  	
	with SMBus(1) as bus:
    # Write a block of 8 bytes to address 80 from offset 0
    	data = [1, 2, 3, 4, 5, 6, 7, 8]
    	bus.write_i2c_block_data(80, 0, data)
	```
- Documentación general de smbus2: [https://github.com/kplindegaard/smbus2/tree/master](https://github.com/kplindegaard/smbus2/tree/master)

