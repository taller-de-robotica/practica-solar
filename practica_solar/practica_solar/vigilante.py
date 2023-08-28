import cv2
import json
import numpy as np
from array import array

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from rclpy.node import Node
from std_msgs.msg import String
from solar_interfaces.srv import DeliverImg


_image_size = (1024, 1792) # Parametro de la dimension de las imagenes
class VigilanteSubscriber(Node):

    def __init__(self):
        super().__init__('vigilante')
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.subscription = self.create_subscription(
                                                String,                 # Tipo de mensaje
                                                'solar_panel_watcher',  # Nombre del topico TODO
                                                self.listener_callback, # Metodo que maneja el evento 
                                                10,                     # Tamaño máximo de cola
                                                callback_group=self.callback_group)
        
        self.subscription  

    def send_poly_dust_request(self, id):
        '''Envia una peticion a un servicio que use la interfaz
        DeliverImg y espera la respuesta.
        '''
        poly_dust_client = self.create_client(DeliverImg ,        # Tipo de peticion
                                            'poly_dust_service')  # Nombre del servicio al cual pedir TODO
        
        # Esperamos a que el servicio este disponible
        while not poly_dust_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('poly_dust_server no disponible, Esperando ...')

        # Creamos la peticion
        poly_dust_request = DeliverImg.Request()
        poly_dust_request.photo_id = id

        # Mandamos la peticion
        return poly_dust_client.call(poly_dust_request)

    def proccess_poly_dust_response(self, response, debug = False):
        ''' Procesa la informacion recibida (bytes) desde el servicio de poly_dust

        '''
        try:
            if response.photo != array('B'):
                # Convertir el objeto array a una matriz NumPy del tipo uint8
                reconstructed_bytes = bytes(response.photo)
                reconstructed_np_array = np.frombuffer(reconstructed_bytes, dtype=np.uint8)

                image_array = np.reshape(reconstructed_np_array, _image_size)
            
                image = cv2.resize(image_array,(_image_size[1]//2, _image_size[0]//2))
                if debug:
                    cv2.imshow("poly_dust_client",  image )
                    cv2.waitKey(100)

        except Exception as ex:
            self.get_logger().warning(f'{ex}')

    def listener_callback(self, msg):
        
        self.get_logger().info('I heard: "%s"' % msg.data)
        try:
            # Recuperamos la informacion desde lo que escuchamos
            data = json.loads(msg.data)

            # Podemos acceder al id como:
            photo_id = data.get('id')
            resultados = data.get('results')
            # Podemos verificar si hay informacion
            if resultados:
                self.get_logger().info('Veo un panel')
            #     raise Exception('La informacion no tiene id')
            
            # # TODO ¿Como sabemos que hay un panel solar?
            
            # # TODO ¿Como sabemos si el panel solar esta limpio o sucio?

            # # Asi podemos invocar al servicio de poly_dust
            # photo =  self.send_poly_dust_request(photo_id)
            # self.proccess_poly_dust_response(photo, debug=False)

        except Exception as ex:
            self.get_logger().warning(f'{ex}')


def main(args=None):
    rclpy.init()
    try:
        print('Iniciando')
        vigilante_subscriber = VigilanteSubscriber()
        executor = MultiThreadedExecutor()
        executor.add_node(vigilante_subscriber)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            vigilante_subscriber.destroy_node()

    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()
   


if __name__ == '__main__':
    main()