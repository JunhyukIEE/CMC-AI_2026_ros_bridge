import socket
import struct

def parse_ego_vehicle_status(data):
    """
    MORAI Ego Vehicle Status UDP 패킷을 파싱합니다.
    패킷 구조는 제공된 문서를 기반으로 해석되었습니다.
    """
    try:
        # 패킷 헤더 '#MoraiInfo$' (11바이트) 이후의 데이터를 사용합니다.
        # 실제 데이터는 추가적인 헤더(29바이트) 뒤에 시작하는 것으로 보입니다.
        # header (11) + packet_id (4) + simulation_mode (1) + control_mode_and_gear (4) + timestamp (8) + something (1) = 29
        # 실제 데이터 시작 오프셋을 29로 가정합니다.
        # 이 오프셋은 경험적인 추정치이며, 실제 시뮬레이터 버전에 따라 다를 수 있습니다.
        
        # 데이터 시작점이 가변적일 수 있으므로, 알려진 문자열 패턴 ('A219')으로 시작점을 찾아봅니다.
        # 'A219AS305665' 문자열이 Link ID일 가능성이 높습니다.
        # Link ID 필드의 시작은 114바이트 오프셋입니다 (아래 구조체 포맷 기준).
        # string_offset = data.find(b'A219')
        # start_offset = string_offset - 114

        # 위 방식 대신, 알려진 패킷 구조에서의 고정 오프셋을 사용해봅니다.
        # #MoraiInfo$ (11) + 200 (4) + 12 바이트 0 = 27 바이트 헤더로 가정
        # 그 다음 CtrlMode(1), Gear(1), signed_vel(4)... 순서로 가정합니다.
        
        header_size = 28 # #MoraiInfo$ (11) + unknown (17) 
        
        # Little-endian (<)을 기준으로 파싱
        # b: signed char (1), f: float (4), i: int (4), 3f: float 3개, 38s: 38-byte string
        # x는 패딩(공백)을 의미합니다.
        format_string = '< 8x i i f f 3f f f f 3f 3f 3f 3f 3f f 38s'
        
        # Timestamp (8) + Ctrl/Gear(2) + padding(2) = 12 바이트를 건너뛰고 속도부터 읽습니다.
        # Timestamp(8) + CtrlMode(1) + Gear(1) + Padding(2) = 12
        offset = 12 
        
        # 1. 속도 (km/h)
        velocity = struct.unpack('<f', data[offset:offset+4])[0]
        
        # 2. 조향각 (deg)
        # 속도부터 조향각까지의 바이트 거리: 4+4+4+12+4+4+4+12+12+12+12+12 = 92
        steer_offset = offset + 92
        steering = struct.unpack('<f', data[steer_offset:steer_offset+4])[0]
        
        return velocity, steering

    except (struct.error, IndexError) as e:
        # 데이터가 너무 짧거나 형식이 맞지 않으면 None을 반환
        # print(f"패킷 파싱 오류: {e}, 데이터 길이: {len(data)}")
        return None, None

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_address = ('192.168.0.37', 9000)
    print(f"{udp_address} 에서 MORAI 데이터 수신 대기 중...")

    try:
        sock.bind(udp_address)
        while True:
            data, addr = sock.recvfrom(1024)
            
            # '#MoraiInfo$' 헤더가 있는지 확인
            if data.startswith(b'#MoraiInfo$'):
                # 헤더(11바이트)와 추가적인 식별자(17바이트)를 제외한 실제 데이터 부분만 파서에 전달
                payload = data[28:]
                velocity, steering = parse_ego_vehicle_status(payload)
                
                if velocity is not None and steering is not None:
                    print(f"차량 속도: {velocity:.2f} km/h, 조향각: {steering:.2f} deg")
                else:
                    print("데이터 수신, 그러나 파싱 실패. 패킷 구조 확인 필요.")
            else:
                print("수신된 데이터가 MORAI 포맷이 아닙니다.")

    except OSError as e:
        print(f"오류: {e}")
    except KeyboardInterrupt:
        print("\n프로그램을 종료합니다.")
    finally:
        sock.close()

if __name__ == '__main__':
    main()
