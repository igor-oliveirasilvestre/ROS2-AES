import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rclpy.service import Service
from std_srvs.srv import Trigger
import random

class SimulaSensor(Node):
    def __init__(self):
        super().__init__('simula_sensor')
        
        # Publicador para os dados do sensor
        self.publicador_sensor = self.create_publisher(Float64, 'dados_sensor', 10)
        self.publicador_media_movel = self.create_publisher(Float64, 'media_movel_sensor', 10)
        
        # Timer para gerar e publicar dados do sensor a cada segundo
        self.temporizador = self.create_timer(1.0, self.publicar_dados_sensor)
        
        # Armazena os dados do sensor
        self.valores_sensor = []  # Ultimos 5 valores para calculo da media movel
        self.historico_valores = []  # Ultimos 64 valores do sensor

        # Servicos para obter o historico e zerar dados
        self.servico_obter_historico = self.create_service(Trigger, 'obter_historico_sensor', self.obter_historico_sensor)
        self.servico_zerar_dados = self.create_service(Trigger, 'zerar_dados_sensor', self.zerar_dados_sensor)

    def publicar_dados_sensor(self):
        # Gera um valor simulado para o sensor
        valor_sensor = random.uniform(0.0, 100.0)
        
        # Adiciona o valor ao historico dos ultimos 64 valores
        self.historico_valores.append(valor_sensor)
        if len(self.historico_valores) > 64:
            self.historico_valores.pop(0)

        # Armazena o valor para a media movel dos ultimos 5 valores
        self.valores_sensor.append(valor_sensor)
        if len(self.valores_sensor) > 5:
            self.valores_sensor.pop(0)

        # Calcula a media movel dos ultimos 5 valores
        media_movel = sum(self.valores_sensor) / len(self.valores_sensor)
        
        # Publica o valor atual do sensor e a media movel
        self.publicador_sensor.publish(Float64(data=valor_sensor))
        self.publicador_media_movel.publish(Float64(data=media_movel))
        
        self.get_logger().info(f'Valor do Sensor: {valor_sensor:.2f} | Media Movel: {media_movel:.2f}')

    def obter_historico_sensor(self, request, response):
        # Retorna o historico dos ultimos 64 valores
        response.success = True
        response.message = f"Ultimos 64 valores do sensor: {self.historico_valores}"
        return response

    def zerar_dados_sensor(self, request, response):
        # Zera o historico de dados do sensor
        self.historico_valores.clear()
        self.valores_sensor.clear()
        response.success = True
        response.message = "Os dados do sensor foram zerados"
        return response

def main(args=None):
    rclpy.init(args=args)
    simula_sensor = SimulaSensor()
    rclpy.spin(simula_sensor)
    simula_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
