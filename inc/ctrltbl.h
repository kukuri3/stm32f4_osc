#ifndef _CTRLTBL_H_
#define _CTRLTBL_H_

typedef
//		union {
//	  uint8_t BYTE[2048];
  struct {
//コントロールテーブルの書き方
//型 (空白) 変数名; (空白)
//説明 (空白) [単位] (空白)    種類{order, static_param, dynamic_param, present}
//start_table コントロールテーブルの開始
	int8_t ref_drive_mode;	//制御モード 0:制御なし 1:電圧 2:電流 3:速度 4:位置 5:速度+位置
	float	ref_current_goal;	//電流指令値_電流制御モードでの指令値。	[A]	order
	float	ref_voltage_goal;	//電圧指令モードの電圧値指令値[V]
	float	ref_velocity_goal;	//出力軸速度指令値_出力軸の速度目標値。指令値として与える。	[rad/s]	order
	float	ref_position_goal;	//位置制御目標位置[rad]
	float	ref_position_control_velocity;	//位置目標制御の速度最大値_出力軸の位置目標値。指令値として与える。	[rad/s]	order
	int64_t	present_position_pulse;	//モータ軸位置[pulse]_モータ軸の位置をパルス数で表したもの。	[pulse]	present
	int64_t	present_position_pulse_pre;	//一つ前のモータ軸位置[pulse]_モータ軸の位置をパルス数で表したもの。	[pulse]	present
	int64_t present_position_pulse_diff;		//モター軸位置の差分[pulse/s]
	float	present_position;	//モータ軸位置[rad]
	int64_t	present_position_pulse2;	//モータ軸位置[pulse]_モータ軸の位置をパルス数で表したもの。	[pulse]	present
	int64_t	present_position_pulse_pre2;	//一つ前のモータ軸位置[pulse]_モータ軸の位置をパルス数で表したもの。	[pulse]	present
	int64_t present_position_pulse_diff2;		//モター軸位置の差分[pulse/s]
	float	present_position2;	//基準エンコーダ位置[rad]
	float	present_pulse_velocity;	//モータ軸速度[pulse/s]_モータ軸の位置に定数とギア比をかけて出力軸の位置で表したもの	[pulse/s]	present
	float	present_velocity;	//モータ軸速度[rad/s]
	int16_t	const_encoder_pulse;	//エンコーダのパルス数(逓倍前。内部で4逓倍している)(512~1000)_present_raw_mr_motor_velocityに定数とギア比をかけて出力軸の速度で表したもの	[pulse/rev]	static_param
	int16_t	const_encoder_pulse2;	//基準エンコーダのパルス数(逓倍前。内部で4逓倍している)(512~1000)_present_raw_mr_motor_velocityに定数とギア比をかけて出力軸の速度で表したもの	[pulse/rev]	static_param
	int16_t	ref_duty_goal;	//電圧指令モードのduty指令値_電圧指令モードでのduty指令値(0～1000)。	[‰digit]	order
	float	present_duty;	//現在出力しているduty値_現在出力されているduty値(0～1000)。	[‰digit]	present
	float const_duty_limit;	//出力するdutyのリミット(~1000)_電圧指令モードでduty_goalを出力する前の制限値(0～1000)。	[‰digit]	static_param
	int16_t present_current_sensor_offset;	//現在の電流センサのオフセット(ADCの生値)。	[digit]
	float	present_current;	//現在電流_現在モータに流れている電流値を電流センサで測ったもの。	[mA]	present
	float	const_current_limit;	//電流制御リミット_出力する電流の上限値。	[mA]	static_param
	float	present_current_goal;	//目標電流(リミット後)_current_goalをcurrent_limitで制限したもの。	[mA]	present
	float	const_current_pgain;	//電流比例ゲイン(~30~)_電流制御における比例ゲイン。	[digit/mA]	dynamic_param
	float	const_current_igain;	//電流積分ゲイン(~300~)_電流制御における積分ゲイン。	[digit/(mA・s)]	dynamic_param
	float	const_current_ierr_max;	//電流積分誤差最大値(~10000~)_電流積分誤差最大値。	[uA・sec]	static_param
	float	present_current_perr;	//電流比例誤差_現在の電流比例誤差。	[mA]	present
	float	present_current_ierr;	//電流積分誤差_現在の電流積分誤差。	[uA・sec]	present
	float	const_velocity_igain;	//速度制御積分ゲイン(~5000~)_速度制御における積分ゲイン。	[uA/mrad]	dynamic_param
	float	const_velocity_pgain;	//速度制御比例ゲイン(~20000~)_速度制御における比例ゲイン。	[uA/(mrad/s)]	dynamic_param
	float	present_velocity_perr;	//速度比例誤差_現在の速度比例誤差。	[rad/s]	present
	float	present_velocity_ierr;	//速度積分誤差_現在の速度積分誤差。	[rad]	present
	float	const_velocity_ierr_max;	//速度積分誤差最大値(~1000000~)_速度積分誤差velocity_ierrの絶対値の最大値。	[urad]	static_param
	float	present_position_perr;	//目標位置比例誤差_現在の位置比例誤差。	[urad]	present
	float	present_position_ierr;	//目標位置積分誤差_現在の位置積分誤差。	[urad/s]	present
	float	present_position_derr;	//目標位置微分誤差_現在の位置微分誤差。	[urad/s]	present
	float	present_velocity_goal_by_poscon;	//位置制御ループによる出力軸目標速度_位置制御により求まった速度制御の目標値。この値が速度制御ループに渡される。	[urad/s]	present
	float	const_position_pgain;	//位置制御比例ゲイン(~100~)_位置制御比例ゲイン。	[(mrad/s)/mrad]	dynamic_param
	float	const_position_dgain;	//位置制御微分ゲイン(0~)_位置制御微分ゲイン。	[(urad/s)/(mrad/s)]	dynamic_param
	float	present_velocity_goal_by_poscon_lim;	//実際に制御に使われる位置目標制御の速度最大値_実際に制御に使われる出力軸の位置指令値。	[urad/s]	present
	int16_t present_enc_raw;
	int16_t present_enc2_raw;
	uint16_t	present_magenc_raw;	//磁気エンコーダの生値
	float	present_magenc_pos;	//磁気エンコーダの位置
	float	const_magenc_offset;	//オフセット
	uint8_t present_magenc_led;	//磁気エンコーダのステータスLEDバイトの値
	uint8_t present_magenc_mem_control;	//磁気エンコーダのステータスMEM_CONTROLの値
	uint8_t present_magenc_sts8;	//磁気エンコーダのステータスSTS8の値
	uint16_t present_magenc_angoffset;	//磁気エンコーダのレジスタANG_OFFSETの値
} TControlTable;

#endif  // include ctrltbl.h
