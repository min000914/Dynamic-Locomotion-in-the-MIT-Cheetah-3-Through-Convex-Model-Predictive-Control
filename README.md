# Dynamic-Locomotion-in-the-MIT-Cheetah-3-Through-Convex-Model-Predictive-Control

[MPC논문pdf](https://dspace.mit.edu/bitstream/handle/1721.1/138000/convex_mpc_2fix.pdf)
[로봇몸체논문pdf](https://dspace.mit.edu/bitstream/handle/1721.1/126619/IROS.pdf)

# 프로젝트 설계

### 1. 설정 및 초기화 모듈 (`Main.m`)

로봇의 스펙과 제어기의 파라미터들을 세팅하고 초기값을 설정합니다.

* **로봇 물리 파라미터:** 질량($m$), 관성 모멘트 텐서($I$), 초기 위치 및 자세, 로봇의 크기(길이, 너비, 높이), 중력가속도
* **보행 파라미터:** 걸음걸이 종류, MPC 제어 주기, MPC Horizon, 전체 보행 주기, 발이 땅에 닿는 시간, 발이 공중에 뜨는 시간
* **시뮬레이션 파라미터:** 시뮬레이션 구동 총 시간, 단위 타임 스텝($dt$)
* **MPC 상태 오차 가중치:** 로봇이 목표 위치, 속도, 자세(Roll, Pitch, Yaw)에 대해 무엇이 우선인지 결정하는 값
* **MPC 제어 입력 가중치:** 지면 반력을 계산할 때 힘의 사용량을 결정하는 값, 각 발이 낼 수 있는 힘의 최소/최대값, 마찰계수

### 2. 궤적 및 보행 생성 모듈


* 발의 궤적을 걸음걸이 종류에 따라 계획 및 계산하는 모듈, 식(33)
* 몸통(무게중심)을 커맨드를 바탕으로 Horizon 동안 움직여야할 궤적을 생성
  
> "All parameters are commanded directly by the robot operator except for yaw and xy-position, which are determined by integrating the appropriate velocities. The other states (roll, pitch, roll rate, pitch rate, and z-velocity) are always set to 0."

# 주요 파라미터
<로봇 논문 발췌> 

<img width="479" height="324" alt="Image" src="https://github.com/user-attachments/assets/7d2f05d2-1d84-4814-89f6-32ab27b6345b" /> 

<MPC논문 발췌>

<img width="603" height="361" alt="Image" src="https://github.com/user-attachments/assets/42fe42ce-5103-477a-a026-950fc61c6e45" />

# 식 내용 정리
## Swing Leg Control

(1)

발이 공중에 있는 동안 월드 좌표계 기준의 발 궤적을 계산하고 이를 tracking 합니다. 이때 목표 궤적을 따라가기 위해 각 다리의 모터가 내야 하는 토크 $\tau_i$는 위치/속도 오차를 줄이는 Feedback 제어와 동역학 특성을 예측하는 Feedforward 제어의 합으로 계산됩니다.


$$\tau_{i} = J_{i}^{\top}[K_{p}(_Bp_{i,ref} - _Bp_{i}) + K_{d}(_Bv_{i,ref} - {}_Bv_{i})] + \tau_{i,ff}$$ 


- $\tau_{i}$: $i$번째 다리의 관절 토크
- $K_p$, $K_d$: 위치와 속도 오차를 줄이기 위한 비례 및 미분 게인 행렬
- 발끝의 현재 위치($Bp_i$)와 속도($Bv_i$) 목표치($Bp_{i,ref}$ , $Bv_{i,ref}$)와의 차이를 계산한 뒤, 자코비안 전치 행렬($J_i^\top$)을 곱해 이를 각 관절 모터가 내야 할 토크로 변환
- $\tau_{i,ff}$: FeedForward 제어값으로 식 (2)에서 설명

(2)

다리를 휘두를 때 발생하는 관성과 중력을 미리 계산해 더해주는 $\tau_{i,ff}$ 항이 추가됩니다.

$$\tau_{i,ff} = J_i^\top \Lambda_i (_Ba_{i,ref} - \dot{J}_i\dot{q}_i) + C_i\dot{q}_i + G_i$$

- $G_i$: 중력 보상
- $C_i\dot{q}_i$: 코리올리 및 원심력
- $\Lambda_i$ $(Ba_{i,ref} - \dot{J}_i\dot{q}_i)$: 3D공간에서의 F=ma
- $\Lambda_i$: 관절에 자세에 따라 느껴지는 다리의 Apparent mass
- $Ba_{i,ref}$: 목표가속도
- $-\dot{J}_i\dot{q}_i$: 모터들이 일정한 속도로 회전($\dot{q}_i$)해도 관절의 각도에 따라 발끝의 속도가 달라지니 이러한 기구학적 변화에 따른 가속도 보정항, 이것을 목표 가속도에서 빼주어 순수하게 원하는 가속력만 모터에 전달할 수 있습니다.
- $J_i^\top$: 발끝의 힘을 각 관절의 힘으로 변환

(3)

식 (1)에서 $K_p$값을 Apparent mass, Natural Frequency에 따라 결정합니다.

$$K_{p,i} = \omega_i^2 \Lambda_{i,i}$$

1차원 질량-스프링에서 질량(m), 스프링 상수(k)인 시스템의 자연진동수는 $\omega_n=\sqrt{\frac{k}{m}}$ 입니다.

따라서 원하는 $\omega_n$를 만들려면 $k = m\omega_n^2$ 이 되고, 여기에 $K_p$와 $\Lambda_i$를 대입합니다.



