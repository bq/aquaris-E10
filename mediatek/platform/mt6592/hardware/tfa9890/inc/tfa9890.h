void tfa9890_SpeakerOn(void);
void tfa9890_SpeakerOff(void);
void tfa9890_setSamplerate(int sRate);
int tfa9890_init(int sRate);
int tfa9890_cali(int spk,int sRate,float* flag);
//int tfa9890_deinit(void);
int tfa9890_EQset(int mode);

