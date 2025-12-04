# STM32F411RE_Vector-Intake-Odor-Detector(벡터흡입식 냄새탐색기)
Created by 전민서, 한정수

# Motivation
<img width="1346" height="639" alt="image" src="https://github.com/user-attachments/assets/852258aa-cea3-48d2-9ae3-1a6aaf4e25bc" />
Odor spreads according to airflow, so relying solely on absolute concentration measurements makes it difficult to accurately identify the true odor source. Single-sensor systems are especially vulnerable to time delay, turbulence, and environmental variations, which can lead to unstable and unreliable tracking performance.

To overcome these limitations, I adopted a “vector-intake” approach that actively controls airflow while inhaling and measuring odor. By manipulating multiple fans to generate directional airflow and comparing the resulting instantaneous concentration changes (ΔC), the system captures not just absolute concentrations but also the concentration gradient shaped by airflow.

This enables more robust and quantitative estimation of odor direction and distance even in complex environments. Developing a new structure capable of such vector-based odor tracking is the core motivation behind this device.




<img width="1127" height="551" alt="image" src="https://github.com/user-attachments/assets/fa4c34a5-3ab1-4ec0-855d-7b863f26d698" />
This system performs a binary search across multiple airflow vector directions and measures the odor intensity in each direction using the SGP30 gas sensor. The direction that yields the highest odor concentration is identified and selected as the estimated odor source.


The figures below illustrate the vector functions used in the system.
<img width="1735" height="798" alt="image" src="https://github.com/user-attachments/assets/0924e90c-b89d-427d-99ee-89d982a4ac49" />



# How to make? STM32MX
<img width="1672" height="608" alt="image" src="https://github.com/user-attachments/assets/c9f81259-819e-4f50-8419-72257908f725" />

<img width="538" height="498" alt="image" src="https://github.com/user-attachments/assets/199ffc9c-793d-4d6b-9b15-563ac0a2a4a3" />

<img width="750" height="741" alt="image" src="https://github.com/user-attachments/assets/6ad2e8f3-b8a3-4053-ad44-2171982c8435" />

<img width="760" height="749" alt="image" src="https://github.com/user-attachments/assets/74bd6eeb-a9c7-4550-85cf-4809e1fef1bf" />




