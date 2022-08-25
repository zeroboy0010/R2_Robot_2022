
#include "omni_wheel.h"
#include "tim.h"

float wheel2(float vx, float vy, float omega){
	return -1.0 * (0.7071/wheel_r)*(vx + vy - 2.0 * wheel_D * omega);
}

float wheel3(float vx, float vy, float omega){
	return -1.0 * (0.7071/wheel_r)*(vx - vy -2.0 * wheel_D * omega);
}

float wheel4(float vx, float vy, float omega){
	return -1.0 * (0.7071/wheel_r)*(-vx - vy -2.0 * wheel_D * omega);
}

float wheel1(float vx, float vy, float omega){
	return -1.0 * (0.7071/wheel_r)*(-vx + vy - 2.0 * wheel_D * omega);
}

float Motors_RPS(int j, float SampleTime, float N_round){
	new_count[Motor1] = Enc_count[0];
	new_count[Motor2] = Enc_count[1];
	new_count[Motor3] = Enc_count[2];
	new_count[Motor4] = Enc_count[3];
	count_state[Motor1] =! dir[0];
	count_state[Motor2] =! dir[1];
	count_state[Motor3] =! dir[2];
	count_state[Motor4] =! dir[3];

	if(count_state[j]){
		if(new_count[j] <= count[j]){ // Check for counter underflow
			diff[j] = count[j] - new_count[j];
		}else{
			diff[j] = (65536 - new_count[j]) + count[j];
		}
			speedM[j] = (float)diff[j] * 1000.0f / (N_round * SampleTime) * -1.0;
	}
	else{
		if(new_count[j] >= count[j]){// Check for counter overflow
			diff[j] = new_count[j] - count[j];
		}else{
			diff[j] = (65536 - count[j]) + new_count[j];
		}
		speedM[j] = (float)diff[j] * 1000.0f / (N_round * SampleTime);
	}

	rdps[j] = 2.0f * pi *speedM[j];
	count[j] = new_count[j];

	return rdps[j];
}

uint16_t encoder(int i){
	if(nowA[i] != lastA[i]){
		lastA[i] = nowA[i];
		if(lastA[i] == 0){
			if(nowB[i] == 0){
				dir[i] = 0;
				cnt[i]--;
			}else{
				dir[i] = 1;
				cnt[i]++;
			}
		}else{
			if(nowB[i] == 1){
				dir[i] = 0;
				cnt[i]--;
			}else{
				dir[i] = 1;
				cnt[i]++;
			}
		}
	}
	if(nowB[i] != lastB[i]){
		lastB[i] = nowB[i];
		if(lastB[i] == 0){
			if(nowA[i] == 1){
				dir[i] = 0;
				cnt[i]--;
			}else{
				dir[i] = 1;
				cnt[i]++;
			}
		}else{
			if(nowA[i] == 0){
				dir[i] = 0;
				cnt[i]--;
			}else{
				dir[i] = 1;
				cnt[i]++;
			}
		}
	}
	return cnt[i];
}
