import xacro, os   
import xml.etree.ElementTree as ET

from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

    # Declara el argumento 'urdf_file' que proporciona el nombre del archivo que
    # contiene el codigo URDF de nuestro modelo. 
    urdf_file_arg = DeclareLaunchArgument(
        "urdf_file", default_value='', description='Name of URDF file.'
    )
 
    return LaunchDescription([
        urdf_file_arg,
        OpaqueFunction(function=startup_nodes)
    ])

def startup_nodes(context, *args, **kwargs):

    # Obtiene el valor del argumento 'urdf_file'.
    urdf_file = LaunchConfiguration('urdf_file').perform(context)

    # Genera una cadena de texto que contene el código URDF.
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    urdf_code = doc.toxml()

    # Genera una estructura de arbol con la información XML del codigo URDF.
    tree = ET.ElementTree(ET.fromstring(urdf_code))
    root = tree.getroot()

    # Recorre el arbol para crear una lista con todos los enlaces presentes en el modelo URDF.
    links = []
    for item in root.iter('link'):
        links.append(item.attrib['name'])
        print(item.tag, ": ", item.attrib['name'])

    # Obtiene el enlace raiz. Para ello elimina de la lista los enlaces que aparecen como 
    # secundarios en las articulaciones.
    for item in root.iter('joint'):
        print(item.tag, ": ", item.attrib['name'])
        for sub_item in item.iter('parent'):
            print("  |--- ", sub_item.tag, ": ", sub_item.attrib['link'])
        for sub_item in item.iter('child'):
            print("  |--- ", sub_item.tag, ": ", sub_item.attrib['link'])
            links.remove(sub_item.attrib['link'])

    if len(links)>1:
        print("Error: el archivo URDF contiene más de un enlace raiz.")
        return

    root_link = links[0]
    print("root_link: ", root_link)

    # Obtiene la dirección del archivo rviz2.config que contiene la configuración para RVIZ2.
    rviz2_config = os.path.join(get_package_share_directory('check_joints'), 'config/config.rviz')

    # Crea y configura el nodo RVIZ2, el cual permite visualizar el modelo URDF a partir de
    # la información que proporciona el nodo ROBOT_STATE_PUBLISHER. 
    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_config, '-f', root_link],
        output={'both': 'log'},
    )

    # Crea y configura el nodo JOINT_STATE_PUBLISHER_GUI, el cual permite simular el estado de
    # las articulaciones.
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=['--ros-args', '--log-level', 'error'],
        output={'both': 'log'},
    )

    # Crea y configura el nodo ROBOT_STATE_PUBLISHER, el cual resuelve la cinematica 
    # directa del robot a partir de la información que proporciona el nodo
    # JOINT_STATE_PUBLISHER_GUI.
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}],
        arguments=['--ros-args', '--log-level', 'error'],
        output={'both': 'log'},
    )


    # Devuelve los nodos creados.
    return [node_rviz2, node_robot_state_publisher, node_joint_state_publisher_gui]