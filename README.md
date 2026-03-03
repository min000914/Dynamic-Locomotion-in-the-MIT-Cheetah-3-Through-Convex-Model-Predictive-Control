# Dynamic-Locomotion-in-the-MIT-Cheetah-3-Through-Convex-Model-Predictive-Control

[pdf](https://dspace.mit.edu/bitstream/handle/1721.1/138000/convex_mpc_2fix.pdf)

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



