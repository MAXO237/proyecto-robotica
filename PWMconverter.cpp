#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <cmath>
#include <algorithm>

// Definición de constantes
const double MAX_RAD_PER_SEC = 60.0; // Velocidad angular máxima 60 de base? Necesitaremos ajustar esto con pruebas físicas
const int MAX_PWM_VALUE = 255;      // Valor máximo de PWM para Arduino, sabemos que va de 0-255

class PWMConverter : public rclcpp::Node
{
public:
    PWMConverter() : Node("pwm_converter")
    {
        // -------------- INICIALIZAR I2C --------------------
        const char *busName = "/dev/i2c-1";
        file_i2c = open(busName, O_RDWR);
        if (file_i2c < 0) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo abrir el bus I2C");
        }

        if (ioctl(file_i2c, I2C_SLAVE, 0x08) < 0) {
            RCLCPP_ERROR(this->get_logger(), "No se pudo configurar el esclavo I2C");
        }

       // 1. Suscriptor: Recibe las velocidades angulares de las ruedas
        _vel_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "wheel_velocities", 10,
            std::bind(&PWMConverter::velocity_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "PWM Converter started");
    }

private:

    void enviar_4_pwm(int p0, int p1, int p2, int p3) {
        uint8_t buffer[4];
        buffer[0] = p0;  
        buffer[1] = p1;
        buffer[2] = p2;
        buffer[3] = p3;

        write(file_i2c, buffer, 4);
    }




     // Función de callback que se ejecuta al recibir un mensaje de velocidad.
    void velocity_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 4) {
            RCLCPP_WARN(this->get_logger(), "Mensaje de velocidad con tamaño inesperado: %zu", msg->data.size());
            return;
        }

        auto pwm_msg = std::make_shared<std_msgs::msg::Int16MultiArray>();
        pwm_msg->data.resize(4);

        // Bucle de conversión para cada rueda: FL, RL, FR, RR
        for (size_t i = 0; i < 4; ++i) {
            double rad_per_sec = msg->data[i];

            // Paso 1: Obtener el valor absoluto de la velocidad para asegurarnos que no trabajemos con negativos
            double abs_rad = std::abs(rad_per_sec);

            // Paso 2: Limitar al valor máximo que propusimos, actualmente tenemos que es 60 (ahora solo es un número random jaja, pueden ajustarlo hasta que se hagan pruebas físicas)
            double limited_rad = std::min(abs_rad, MAX_RAD_PER_SEC);

            // Paso 3: convertimos linealmente con PWM = (Valor_Actual / Valor_Máximo_Rad) * Valor Máximo PWM
            int pwm_value = static_cast<int>(
                (limited_rad / MAX_RAD_PER_SEC) * MAX_PWM_VALUE
            );
            
            // Paso 4: Aseguramos que el valor final esté en el rango de los PWM, solo devuelve el valor original, el límite inferior o el superior dependiendo si le falta o se pasa. (no debería de pasar pero se deja como seguridad extra)
            pwm_msg->data[i] = std::clamp(pwm_value, 0, MAX_PWM_VALUE);
        }

        RCLCPP_INFO(this->get_logger(),
            "PWMs publicados: FL=%d, RL=%d, FR=%d, RR=%d",
            pwm_msg->data[0], pwm_msg->data[1], pwm_msg->data[2], pwm_msg->data[3]);


        enviar_4_pwm(
            pwm_msg->data[0],
            pwm_msg->data[1],
            pwm_msg->data[2],
            pwm_msg->data[3]
        );
    }

    int file_i2c;  
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr _vel_subscription;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PWMConverter>());
    rclcpp::shutdown();
    return 0;
}