import unittest
from questao3 import decodificar_mensagem
import binascii

class TestDecodificacaoAES(unittest.TestCase):
    def test_mensagem_decodificada(self):
        # Teste com valores conhecidos
        cifra_hex = "a57fd9725fb53c53d5bd0b56185da50f70ab9baea5a43523b76c03e3eb989a20"
        chave = "thisisasecretkey"
        mensagem_esperada = "Sistemas Embarcados"
        
        # Executando a decodificacao
        mensagem_resultante = decodificar_mensagem(cifra_hex, chave)
        
        # Verificando se a mensagem decodificada corresponde ao esperado
        self.assertEqual(mensagem_resultante, mensagem_esperada)

    def test_chave_incorreta(self):
        # Teste com chave incorreta
        chave = "thisisasecretkey"
        cifra_hex = "a57fd9725fb53c53d5bd0b56185da50f70ab9baea5a43523b76c03e3eb989a20"
        chave_incorreta = "thisisasecretkee"
        
        # Executando a decodificacao com chave incorreta
        mensagem_resultante = decodificar_mensagem(cifra_hex, chave_incorreta)

        #mensagem_resultante = mensagem_resultante.rstrip(b"\x00").decode('utf-8').strip()
        
        # Verificando que a saida nao corresponde a mensagem esperada, confirmando que a chave esta errada
        self.assertNotEqual(mensagem_resultante, "Sistemas Embarcados")

    def test_cifra_invalida(self):
        chave = "thisisasecretkey"
        cifra_invalida = "ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ"
        
        with self.assertRaises(binascii.Error):
            decodificar_mensagem(cifra_invalida, chave)

    def test_chave_curta(self):
        cifra_hex = "a57fd9725fb53c53d5bd0b56185da50f70ab9baea5a43523b76c03e3eb989a20"
        chave_incorreta = "1234"
        
        with self.assertRaises(ValueError):
            decodificar_mensagem(cifra_hex, chave_incorreta)

# Executando os testes
if __name__ == '__main__':
    unittest.main()
