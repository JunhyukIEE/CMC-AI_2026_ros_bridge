# MORAI PC 교체 전후 UDP 타이밍 비교

ROS 없이 실행하는 독립 도구입니다. 브릿지/Autoware와 동시에 실행하며 수신 포트에 bind하지 않습니다.
제어/리셋 패킷을 보내지 않고 tcpdump로 헤더를 관찰합니다. 기본으로 1 Hz ICMP ping만 추가합니다.
Python 표준 라이브러리, 시스템 `tcpdump`, `ping`을 사용합니다. 빌드/설치 등록은 필요 없습니다.

## 실행

브릿지와 시뮬레이터를 켜고 실제 주행 중 측정합니다. 네트워크 인터페이스는
`ip route get 192.168.0.27`의 `dev` 값을 사용하세요.

```bash
cd /home/ljh/workspace/CMC-AI_2026
sudo /usr/bin/python3 src/morai_src/morai_bridge/scripts/measure_udp_latency.py \
  --sim-ip 192.168.0.27 --interface enp8s0 --duration 60 --label current_pc
```

대회 PC로 바꾸면 목적지 IP를 바꾸고 `--label competition_pc`로 같은 시험을 합니다.
센서 해상도/Hz, 장면/경로/속도, Autoware와 RViz/recording 부하, 케이블/스위치 조건도 맞춥니다.
각 조건에서 3회 정도 반복하고 정차뿐 아니라 코너·감속 구간을 포함하세요.
`--no-ping`은 완전 수동 관찰만 합니다. Ctrl-C도 캡처를 종료하고 결과를 저장합니다.

`udp_latency_results/날짜_시각_마이크로초_라벨/`에 매번 새 디렉터리가 만들어집니다.

- `summary.json`: 방향/포트/프로토콜별 Hz, 간격과 age의 평균/p50/p95/p99/max, 긴 공백 수, ping RTT, 캡처 상태.
- `samples.csv`: 각 샘플 도착시각·패킷 timestamp·간격·age. 카메라는 timestamp별 첫 관측 조각 한 줄.
- `headers.pcap`: 128 byte까지만 저장한 패킷 헤더. JPEG 전체나 ROS bag을 저장하지 않습니다.
- `capture.log`: 캡처 오류 및 kernel drop 수. drop이 있으면 측정 누락 가능성이 있습니다.
- `ping.log`: ICMP 응답 시간과 손실률. UDP 손실률이나 제어 적용 시간을 의미하지 않습니다.

예전 캡처를 다시 분석할 수도 있습니다(권한/네트워크 불필요).

```bash
python3 src/morai_src/morai_bridge/scripts/measure_udp_latency.py \
  --sim-ip 192.168.0.27 --pcap /absolute/path/headers.pcap --label recheck
```

## 숫자를 해석하는 방법

**시계부터 맞추세요.** 두 PC를 같은 NTP 기준에 동기화하고, Linux는 `chronyc tracking`,
Windows는 `w32tm /query /status`로 상태를 확인합니다. 측정 중 시계를 수동 변경하지 마세요.

`raw_age_ms = 브릿지 PC의 패킷 캡처시각 - MORAI 패킷 timestamp`입니다.
여기에는 **두 PC 시계 차이 + 시뮬레이터 생성/인코딩/송신 대기 + 네트워크**가 들어갑니다.
시뮬레이터 timestamp가 경과시간/다른 시간축이면 절대 지연으로 해석할 수 없습니다.
미동기 상태의 1.5초 age를 네트워크 지연이라고 해석하지 마세요.
최소 age를 빼서 동기화한 척하지 않습니다.

시계 차이를 독립적으로 측정했다면 `--clock-offset-ms`를 지정할 수 있습니다.
부호는 **브릿지 시계 - 시뮬레이터 시계**입니다. 브릿지가 15 ms 앞서면 `15`를 넣습니다.
`corrected_age_ms = raw_age_ms - offset`이며, 여전히 시뮬레이터 내부 처리시간을 포함합니다.
같은 epoch로 동기화가 검증됐을 때만 `--clock-offset-ms 0`도 의미가 있습니다.

| 항목 | 용도/주의 |
|---|---|
| RX 9000 STATUS | 현 코드의 181 B / payload 152 B 형식만 timestamp 해석. 다른 버전은 age 공란, 주기만 측정 |
| RX 카메라 MOR/BOX | 첫 관측 조각의 age와 프레임 주기. 전체 IP/JPEG 조립·decode·ROS 지연은 포함하지 않음 |
| RX IMU / COLLISION | 현재 브릿지 레이아웃의 sec/nsec 해석. 원천 시간축 확인 필요 |
| NMEA / 기타 UDP | 수신 주기/긴 공백만 측정. 추측으로 epoch timestamp를 만들지 않음 |
| TX 9091 CTRL | 브릿지 NIC 측 송신 주기/공백만 확인. 시뮬레이터 도착·적용 지연은 알 수 없음 |
| ping RTT | 시계 차이와 무관한 ICMP 왕복시간. UDP 처리시간이나 RTT/2 단방향 지연의 보장은 아님 |

카메라 IP 후속 fragment는 해석하지 않으며 첫 fragment만 셉니다. 첫 조각을 놓치거나 캡처가
drop되면 실제보다 age/간격이 길어질 수 있습니다. 일부 알려진 timestamp 외에는 형식을 추측하지 않습니다.
`any` 인터페이스는 중복 캡처될 수 있어 물리 NIC 하나를 권장합니다.

## 대회 대비 확인 순서

1. RX 9000의 최대 간격과 `gaps_over_threshold`를 비교합니다. 현재 sender의
   `velocity_timeout_sec=0.2` 때문에 200 ms 이상 공백은 stale 0/0 출력의 원인이 될 수 있습니다.
   단 캡처 시점과 ROS callback 시점은 다르므로 이 도구만으로 stale 발생을 확정하지는 않습니다.
2. TX 9091 간격이 늘어나면 bridge/Autoware 명령 공급·호스트 부하를 확인합니다.
   주행 중 TX 자체가 없다면 control input/송신 경로를 먼저 확인합니다.
3. ping RTT는 비슷한데 동기화된 MOR age만 커지면 새 시뮬레이터 PC의 센서 생성·인코딩 부하를 확인합니다.
4. 패킷 age/주기는 정상인데 ROS 영상이나 제어 반응이 느리면 bridge decode/DDS/Autoware 단계의 별도 측정이 필요합니다.

`--gap-ms 200`은 모든 스트림에 공통 적용한 공백 표시 기준일 뿐 합격 기준이 아닙니다.
충돌 이벤트/저주기 GNSS는 긴 간격이 정상일 수 있습니다. packet-loss 비율은 sequence 정보 없이 추정하지 않습니다.
실제 제어 전달 단방향 시간을 재려면 시뮬레이터 PC에서도 동기화된 수신 캡처를 떠서 동일 9091 패킷을
매칭해야 합니다. 반복되는 동일 명령은 매칭이 모호할 수 있고, 수신 이후 제어 적용 시간은 별도 계측이 필요합니다.

## 네트워크 없이 검증

```bash
python3 src/morai_src/morai_bridge/scripts/test_measure_udp_latency.py
```
