// =============================================================================
//	RS232C �p���W���[��
//
//					Filename: rs_comm.c
//
// =============================================================================
//		Ver 1.0.0		2012/11/01
// =============================================================================

//�V���A���ʐM�p
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include "leptrino/rs_comm.h"

#define MAX_BUFF		10
#define MAX_LENGTH	255

#define STS_IDLE		0
#define STS_WAIT_STX	1
#define STS_DATA		2
#define STS_WAIT_ETX	3
#define STS_WAIT_BCC	4

static int Comm_RcvF=0;								//��M�f�[�^�L�t���O
static int p_rd=0,p_wr=0;							//��M�����O�o�b�t�@�Ǐo���A�����݃|�C���^
static int fd=0;										//
static int rcv_n=0;									//��M������

static UCHAR delim;									//��M�f�[�^�f���~�^
static UCHAR rcv_buff[MAX_BUFF][MAX_LENGTH];	//��M�����O�o�b�t�@
static UCHAR stmp[MAX_LENGTH];						//

struct termios tio;									//�|�[�g�ݒ�\����


// ----------------------------------------------------------------------------------
//	�f�o�C�X�I�[�v��
// ----------------------------------------------------------------------------------
//	����	: dev .. �V���A���|�[�g
//	�߂�l	: ���펞:0   �G���[��:-1
// ----------------------------------------------------------------------------------
int Comm_Open(char *dev)
{
	//���ɃI�[�v�����Ă���Ƃ��͈�x����
	if (fd != 0) Comm_Close();
	//�|�[�g�I�[�v��
	fd = open(dev, O_RDWR | O_NDELAY | O_NOCTTY);
	if (fd < 0) return NG;
	//�f���~�^
	delim=0;
		
	return OK;
}

// ----------------------------------------------------------------------------------
//	�f�o�C�X�N���[�Y
// ----------------------------------------------------------------------------------
//	����	: non
//	�߂�l	: non
// ----------------------------------------------------------------------------------
void Comm_Close()
{
	if (fd > 0) {
		close(fd);
	}
	fd=0;
		
	return;
}

// ----------------------------------------------------------------------------------
//	�|�[�g�ݒ�
// ----------------------------------------------------------------------------------
//	����	: boud   .. �{�[���[�g 9600 19200 ....
//			: parity .. �p���e�B�[ 
//			: bitlen .. �r�b�g��
//			: rts    .. RTS����
//			: dtr    .. DTR����
//	�߂�l	: non
// ----------------------------------------------------------------------------------
void Comm_Setup(long baud ,int parity ,int bitlen ,int rts ,int dtr ,char code)
{
	long brate;
	long cflg;
	
	switch (baud) {
	case 2400  :brate=B2400;  break;
	case 4800  :brate=B4800;  break;
	case 9600  :brate=B9600;  break;
	case 19200 :brate=B19200; break;
	case 38400 :brate=B38400; break;
	case 57600 :brate=B57600; break;
	case 115200:brate=B115200;break;
	case 230400:brate=B230400;break;
	case 460800:brate=B460800;break;
	default    :brate=B9600;  break;
	}
	//�p���e�B
	switch (parity) {
	case PAR_NON:cflg=0;					 break;
	case PAR_ODD:cflg=PARENB | PARODD;break;
	default     :cflg=PARENB;			 break;
	}
	//�f�[�^��
	switch (bitlen) {
	case 7 :cflg |= CS7;break;
	default:cflg |= CS8;break;
	}
	//DTR
	switch (dtr) {
	case 1 :cflg &= ~CLOCAL;break;
	default:cflg |= CLOCAL; break;
	}
	//RTS CTS
	switch (rts) {
	case 0 :cflg &= ~CRTSCTS;break;
	default:cflg |= CRTSCTS; break;
	}
	
	//�|�[�g�ݒ�t���O
	tio.c_cflag = cflg | CREAD;
	tio.c_lflag = 0;
	tio.c_iflag = 0;
	tio.c_oflag = 0;
	tio.c_cc[VTIME] = 0;
	tio.c_cc[VMIN]  = 0;
	
	cfsetspeed(&tio, brate);	
	tcflush( fd, TCIFLUSH);				//�o�b�t�@�̏���
	tcsetattr( fd, TCSANOW , &tio);		//�����̐ݒ�
	
	delim=code;								//�f���~�^�R�[�h
	return;
}

// ----------------------------------------------------------------------------------
//	�����񑗐M
// ----------------------------------------------------------------------------------
//	����	: buff .. ������o�b�t�@
//			: l    .. ���M������
//	�߂�l	: 1:OK -1:NG
// ----------------------------------------------------------------------------------
int Comm_SendData( UCHAR *buff, int l)
{
	if (fd <= 0 ) return -1;
	
	write( fd, buff, l);
	
	return OK;
}

// ----------------------------------------------------------------------------------
//	��M�f�[�^�擾
// ----------------------------------------------------------------------------------
//	����	: buff .. ������o�b�t�@
//	�߂�l	: ��M������
// ----------------------------------------------------------------------------------
int Comm_GetRcvData(UCHAR *buff)
{
	int l = rcv_buff[p_rd][0];
	
	if ( p_wr == p_rd ) return 0;
	
	memcpy(buff, &rcv_buff[p_rd][0], l);
	p_rd++;
	if (p_rd >= MAX_BUFF) p_rd=0;
	
	l=strlen((char*)buff);
	
	return l;
}

// ----------------------------------------------------------------------------------
//	��M�L���m�F
// ----------------------------------------------------------------------------------
//	����	: non 
//	�߂�l	: 0:�Ȃ� 0�ȊO�F����
// ----------------------------------------------------------------------------------
int Comm_CheckRcv()
{
	return p_wr-p_rd;
}

// ----------------------------------------------------------------------------------
//	��M�Ď��X���b�h
// ----------------------------------------------------------------------------------
//	����	: pParam .. 
//	�߂�l	: non
// ----------------------------------------------------------------------------------
unsigned char rbuff[MAX_LENGTH];
unsigned char ucBCC;
void Comm_Rcv(void)
{
	int i,rt=0;
	unsigned char ch;
	static int RcvSts = 0;

	while(1){
		rt=read(fd, rbuff, 1);
		//��M�f�[�^����
		if (rt > 0) {
			rbuff[rt]=0;
			ch=rbuff[0];
			
			switch (RcvSts) {
			case STS_IDLE:
				ucBCC = 0;								/* BCC */
				rcv_n = 0;
				if (ch == CHR_DLE) RcvSts = STS_WAIT_STX;
				break;
			case STS_WAIT_STX:
				if (ch == CHR_STX) {					/* STX������Ύ��̓f�[�^ */
					RcvSts = STS_DATA;
				} else {								/* STX�łȂ���Ό��ɖ߂� */
					RcvSts = STS_IDLE;
				}
				break;
			case STS_DATA:
				if (ch == CHR_DLE) {					/* DLE������Ύ���ETX */
					RcvSts = STS_WAIT_ETX;
				} else {								/* ��M�f�[�^�ۑ� */
					stmp[rcv_n] = ch;
					ucBCC ^= ch;						/* BCC */
					rcv_n++;
				}
				break;
			case STS_WAIT_ETX:
				if (ch == CHR_DLE) {					/* DLE�Ȃ�΃f�[�^�ł��� */
					stmp[rcv_n] = ch;
					ucBCC ^= ch;						/* BCC */
					rcv_n++;
					RcvSts = STS_DATA;
				} else if (ch == CHR_ETX) {				/* ETX�Ȃ玟��BCC */
					RcvSts = STS_WAIT_BCC;
					ucBCC ^= ch;						/* BCC */
				} else if (ch == CHR_STX) {			/* STX�Ȃ烊�Z�b�g */
					ucBCC = 0;							/* BCC */
					rcv_n = 0;
					RcvSts = STS_DATA;
				} else {
					ucBCC = 0;							/* BCC */
					rcv_n = 0;
					RcvSts = STS_IDLE;
				}
				break;
			case STS_WAIT_BCC:
				if (ucBCC == ch) {						/* BCC��v */
					//�쐬���ꂽ������������O�o�b�t�@�փR�s�[
					memcpy(rcv_buff[p_wr], stmp, rcv_n);
					p_wr++;
					if ( p_wr >= MAX_BUFF ) p_wr=0;
				}
				/* ���̃f�[�^��M�ɔ����� */
				ucBCC = 0;					/* BCC */
				rcv_n = 0;
				RcvSts = STS_IDLE;
				break;
			default:
				RcvSts = STS_IDLE;
				break;
			}
			
			if (rcv_n  > MAX_LENGTH) {
				ucBCC = 0;
				rcv_n = 0;
				RcvSts = STS_IDLE;
			}
		} else {
			break;
		}
		
		//��M�����t���O
		if (p_rd != p_wr) {
			Comm_RcvF=1;
		} else {
			Comm_RcvF=0;
		}
	}
}
