from Crypto.Cipher import AES
import binascii

def decodificar_mensagem(cifra_hex, chave):
    # Convertendo a chave e a cifra de hexadecimal para bytes
    chave_bytes = chave.encode('utf-8')
    cifra_bytes = binascii.unhexlify(cifra_hex)
    
    # Inicializando o modo ECB com a chave de 128 bits
    cipher = AES.new(chave_bytes, AES.MODE_ECB)
    
    # Decodificando a mensagem
    mensagem_decodificada = cipher.decrypt(cifra_bytes)
    
    # Removendo qualquer padding e retornando a mensagem decodificada em formato legivel
    #return mensagem_decodificada.rstrip(b"\x00").decode('utf-8')
    #return mensagem_decodificada.rstrip(b"\x00").decode('utf-8').strip()
    try:
        return mensagem_decodificada.rstrip(b"\x00").decode('utf-8').strip()
    except UnicodeDecodeError:
        mensagem_decodificada = "Erro na decodificação"
        return mensagem_decodificada

# Parametros da questao
cifra_hex = "a57fd9725fb53c53d5bd0b56185da50f70ab9baea5a43523b76c03e3eb989a20"
chave = "thisisasecretkey"

# Decodificando e imprimindo a mensagem
mensagem = decodificar_mensagem(cifra_hex, chave)
print("Mensagem decodificada:", mensagem)
