# Questao 2

Este projeto fornece instruções e arquivos de configuração para criar um ambiente Dockerizado para nossa aplicação. Siga as instruções abaixo para instalar o Docker no Ubuntu.

## Pré-requisitos
- **Sistema Operacional**: Recomendando Ubuntu 22.04 para compatibilidade com ROS2 Humble 

## Instalação do Docker

Siga os passos abaixo para instalar o Docker no seu sistema Ubuntu.

#### Passo 1: Atualize os pacotes e instale as dependências
```bash
sudo apt-get update
sudo apt-get install ca-certificates curl
```
#### Passo 2: Adicione a chave GPG oficial do Docker
```bash
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
```
#### Passo 3: Adicione o repositório do Docker às fontes do Apt
```bash
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
  ```

#### Passo 4: Atualize a lista de pacotes
```bash
sudo apt-get update
```

#### Passo 5: Instale o Docker Engine
```bash
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

#### Passo 6: Verifique a instalação
```bash
sudo docker run hello-world
```


## Configuração do Dockerfile

Para utilizar um Dockerfile específico, siga os passos abaixo:

#### Passo 1: Baixar o Dockerfile

Baixe o arquivo Dockerfile e coloque-o em uma pasta, por exemplo, ~/questao2/. Se você usar outro nome para o diretório, as linhas de comando a seguir precisam ser atualizadas com o novo diretório (substituindo questao2 pelo nome do diretório que você utilizou).

#### Passo 2: Construir a imagem Docker

Abra um terminal no diretório onde o Dockerfile se encontra e execute o comando:

```bash
docker build -t questao2 .
```
#### Passo 3: Executar o container Docker

Após a construção do Docker, execute o seguinte comando para iniciar o container:
```bash
docker run -it \
  --name questao2 \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  questao2
```
#### Passo 4: Iniciar o Docker

Por fim, para iniciar o Docker a partir de um container já existente, use o comando:

```bash
docker run -it questao2
```
## Para buildar os pacotes e dar inicio aos testes
```bash
cd /ros2_ws
colcon build
source install/setup.bash
```

## Rodando pacote 1:
Para executar o monitor de memória RAM basta chamar ele utilizando o ros, de forma:
```bash
ros2 run monitor mempub
```

Para verificar se as mensagens estão sendo publicadas corretamente, em outro terminal acesse o docker:
```bash
docker run -it questao2
```
E execute o seguinte comando para escutar o tópico:

```bash
ros2 topic echo /memory_usage
```


## Rodando pacote 2:

Para executar o simulador do sensor a 1HZ basta chamar utilizando o ros, de forma:

```bash
ros2 run simulasensor sensor
```

Para a interface onde temos os últimos 64 valores do sensor, se pode abrir outro terminal e chamar a interface:

```bash
ros2 service call /obter_historico_sensor std_srvs/srv/Trigger
```
Isso deve retornar os últimos 64 valores do sensor.

Para a interface onde se reseta os dados do sensor, se pode abrir outro terminal e chamar a interface:


```bash
ros2 service call /zerar_dados_sensor std_srvs/srv/Trigger
```
Isso deve limpar os dados armazenados pelo sensor.

## Rodando pacote 3:

Para rodar o server de ações, chame ele por:
```bash
cd /ros2_ws/src/action_tutorials_interfaces/src/ && python3 prime_action_server.py 
```

Para chamar uma ação, em outro terminal, chame por:
```bash
ros2 action send_goal --feedback prime action_tutorials_interfaces/action/Prime "{start: 0}"
```

Essa chamada irá retornar os 10 primeiros números primos a partir do paramêtro start. Se desejado os 10 primeiros o start deve ser passado como 0.
