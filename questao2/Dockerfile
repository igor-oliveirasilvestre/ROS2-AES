# Use uma imagem base compatível com ROS2 Humble
FROM ubuntu:22.04

# Defina variáveis de ambiente para configuração do ROS
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Atualize o sistema e instale pacotes básicos e dependências do ROS2
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    cmake \
    git \
    vim \
    nano \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Configure o locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG = en_US.UTF-8

# Adicione o repositório do ROS2 e a chave GPG
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Instale o ROS2 Humble e ferramentas de desenvolvimento
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-desktop \
    python3-rosdep \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Instale o psutil, necessário para monitorar a memória no Pacote "1"
RUN pip3 install psutil

# Inicialize o rosdep para gerenciar dependências
RUN rosdep init && rosdep update

# Configure o ambiente ROS2 para ser carregado ao iniciar o container
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc


# Defina o diretório de trabalho para o workspace do ROS2
WORKDIR /ros2_ws

# Crie um workspace ROS2
RUN mkdir -p src && cd src

# Clone the repository into a temporary directory
RUN git clone https://github.com/igor-oliveirasilvestre/ROS2-AES.git /tmp/ROS2-AES

# Copy contents from the questao2 folder to /ros2_ws/
RUN cp -r /tmp/ROS2-AES/questao2/* /ros2_ws/

# Clean up by removing the cloned repository
RUN rm -rf /tmp/ROS2-AES

# Copie o código dos pacotes para o workspace ROS2 (adicione os scripts localmente depois de construir a imagem)
COPY . /ros2_ws/src/

# Construa o workspace ROS2
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build"

# Fonte do ambiente ROS2 ao iniciar o container
CMD ["bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash && source install/setup.bash && bash"]

