import socket

def main():
    # UDP 소켓 생성
    # AF_INET: IPv4, SOCK_DGRAM: UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 바인딩할 주소와 포트
    udp_address = ('192.168.0.37', 1578)
    print(f"{udp_address} 에서 UDP 메시지를 기다리는 중...")

    try:
        # 소켓을 주소와 포트에 바인딩
        sock.bind(udp_address)

        while True:
            # 데이터 수신. 1024는 버퍼 크기
            data, addr = sock.recvfrom(1024) 
            print(f"주소 {addr} 로부터 메시지 수신:")
            print(f"  - 원본 데이터 (bytes): {data}")
            try:
                # 데이터를 UTF-8로 디코딩하여 출력 시도
                print(f"  - 디코딩된 데이터 (utf-8): {data.decode('utf-8')}")
            except UnicodeDecodeError:
                print("  - (UTF-8로 디코딩할 수 없는 데이터입니다)")

    except OSError as e:
        print(f"오류: {e}")
        print("포트 1578이 이미 사용 중일 수 있습니다.")
    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다.")
    finally:
        # 소켓 닫기
        sock.close()

if __name__ == '__main__':
    main()
