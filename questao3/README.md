# Questao 3

Este projeto fornece instruções e arquivos para implementar um sistema de decodificação utilizando o algoritmo AES (Advanced Encryption Standard) em modo ECB. A criptografia AES é amplamente utilizada em sistemas embarcados para garantir a segurança de dados. O código inclui testes unitários para validar o funcionamento da decodificação em diferentes cenários, assegurando a robustez do software


## Pré-requisitos

- **python3**
- **pycryptodome**: instalado por :
```bash
pip install pycryptodome
```

## Execução do código decodificador

Com o diretório questão3 deste git baixado ou o repositório clonado:
Para executar o código capaz de realizar a decodificação da mensagem abra um terminal na pasta `questao3/src` e chame:

```bash
python3 questao3.py
```

Esse código deve retornar a seguinte frase codificada em AES modo ECB 
```
a57fd9725fb53c53d5bd0b56185da50f70ab9baea5a43523b76c03e3eb989a20
```
Utilizando a chave:
```
thisisasecretkey
```
E imprime no terminal o resultado da decodificação, que segue:
```
Mensagem decodificada: Sistemas Embarcados
```

## Testes unitários

Os testes unitários estão declarados no arquivo `testeunitario.py`, para executar estes testes podemos novamente abrir um terminal no diretório `questao3/src` e chamar:
```bash
python3 -m unittest discover
```
Que deve, por default, imprimir:
```
Ran 4 tests in 0.001s

OK
```

Os seguintes testes unitários foram programados:
- **test_mensagem_decodificada** : Testa a função `decodificar_mensagem` declarada no arquivo `questao3.py` em AES e modo ECB utilizando a chave e frase corretas.Este teste unitário compara o resultado com o resultado esperado da decodificação `"Sistemas Embarcados"`.
- **test_chave_incorreta** : Testa a função `decodificar_mensagem`, utilizando uma chave incorreta e frase correta, verificando se o resultado é diferente do esperado quando a decodificação ocorre corretamente `"Sistemas Embarcados"`.
- **test_cifra_invalida** : Testa a função `decodificar_mensagem`, utilizando uma frase incorreta  e chave correta, verificando se um erro ocorre durante a decodificação da mensagem.
- **test_chave_curta** : Testa a função `decodificar_mensagem` com uma chave curta menor do que os 16 bytes esperados, verificando se um erro ocorre durante a decodificação da mensagem.


## Geração de falhas nos testes unitários
Estes testes podem ser alterados para gerar falhas, dependendo da natureza de cada teste, as instruções para reproduzir estas falhas seguem:

### test_mensagem_decodificada
Para o teste de mensagem decodificada, podemos alterar a linha 10 de forma a não representar mais a mensagem esperado utilizando a chave e frase correta:
```bash
mensagem_esperada = "Embarcados Sistemas"
```

Se os testes forem rodados usando `python3 -m unittest discover` devemos detectar a seguinte falha:
```bash
AssertionError: 'Sistemas Embarcados' != 'Embarcados Sistemas'
```


### test_chave_incorreta
Para o teste de chave incorreta, podemos alterar a linha 22 de forma a agora representar a chave correta, de forma:
```bash
chave_incorreta = "thisisasecretkey"
```

Se os testes forem rodados usando `python3 -m unittest discover` devemos detectar a seguinte falha:
```bash
AssertionError: 'Sistemas Embarcados' == 'Sistemas Embarcados'
```


### test_cifra_invalida
Para o teste de frase incorreta, podemos alterar a linha 34 de forma a agora representar a frase correta, de forma:
```bash
cifra_invalida = "a57fd9725fb53c53d5bd0b56185da50f70ab9baea5a43523b76c03e3eb989a20"
```

Se os testes forem rodados usando `python3 -m unittest discover` devemos detectar a seguinte falha:
```bash
AssertionError: Error not raised
```

### test_chave_curta
Para o teste de chave curta, podemor alterar a linha 41 de forma a agora representar qualquer chave com 16 bytes, por exemplo:
```bash
chave_incorreta = "123456789abcdefg"
```

Se os testes forem rodados usando `python3 -m unittest discover` devemos detectar a seguinte falha:
```bash
AssertionError: ValueError not raised
```
