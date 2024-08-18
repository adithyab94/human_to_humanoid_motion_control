%% Body BSP parameters
Tb_BSP = readtable("BSP.csv");

l_lower_trunk = 18.3; %cm
l_middle_trunk = 29.4; %cm
m_lower_trunk = Tb_BSP.Mass_kg_(10);
m_middle_trunk = Tb_BSP.Mass_kg_(9);

mi = [Tb_BSP.Mass_kg_(4) Tb_BSP.Mass_kg_(4) Tb_BSP.Mass_kg_(6) Tb_BSP.Mass_kg_(6) Tb_BSP.Mass_kg_(5) Tb_BSP.Mass_kg_(2) Tb_BSP.Mass_kg_(5) Tb_BSP.Mass_kg_(3) Tb_BSP.Mass_kg_(10)+Tb_BSP.Mass_kg_(9) Tb_BSP.Mass_kg_(1) Tb_BSP.Mass_kg_(3) Tb_BSP.Mass_kg_(2) nan nan Tb_BSP.Mass_kg_(1) Tb_BSP.Mass_kg_(8) Tb_BSP.Mass_kg_(7)];
l_list = [25.46 25.46 36.04 36.04 38.22 25.77 38.22 29.4 l_lower_trunk+l_middle_trunk 18.9 29.4 25.77 nan nan 18.9 23.8 22.8]*1e-2;
z_csv_list = Tb_BSP.z_cm_*1e-2;

xg_hip = ((l_middle_trunk*1e-2+z_csv_list(10))*m_lower_trunk+z_csv_list(9)*m_middle_trunk)/(m_lower_trunk+m_middle_trunk);
l_hand = l_list(10);
xg_list = [z_csv_list(4)    z_csv_list(4)      z_csv_list(6)      z_csv_list(6)      z_csv_list(5)    l_list(6)-z_csv_list(2) z_csv_list(5)  l_list(8)-z_csv_list(3)   xg_hip                        l_hand/2       l_list(8)-z_csv_list(3) l_list(12)-z_csv_list(2) 0 0  l_hand/2    z_csv_list(8)   z_csv_list(7)   ];

IxxG = [Tb_BSP.Izz_g_cm_2_(4) Tb_BSP.Izz_g_cm_2_(4) Tb_BSP.Izz_g_cm_2_(6) Tb_BSP.Izz_g_cm_2_(6) Tb_BSP.Izz_g_cm_2_(5) Tb_BSP.Izz_g_cm_2_(2) Tb_BSP.Izz_g_cm_2_(5) Tb_BSP.Izz_g_cm_2_(3) Tb_BSP.Izz_g_cm_2_(10)+Tb_BSP.Izz_g_cm_2_(9) Tb_BSP.Izz_g_cm_2_(1) Tb_BSP.Izz_g_cm_2_(3) Tb_BSP.Izz_g_cm_2_(2) nan nan Tb_BSP.Izz_g_cm_2_(1) Tb_BSP.Izz_g_cm_2_(8) Tb_BSP.Izz_g_cm_2_(7)]*1e-7;
IyyG = [Tb_BSP.Ixx_g_cm_2_(4) Tb_BSP.Ixx_g_cm_2_(4) Tb_BSP.Ixx_g_cm_2_(6) Tb_BSP.Ixx_g_cm_2_(6) Tb_BSP.Ixx_g_cm_2_(5) Tb_BSP.Ixx_g_cm_2_(2) Tb_BSP.Ixx_g_cm_2_(5) Tb_BSP.Ixx_g_cm_2_(3) Tb_BSP.Ixx_g_cm_2_(10)+Tb_BSP.Ixx_g_cm_2_(9) Tb_BSP.Ixx_g_cm_2_(1) Tb_BSP.Ixx_g_cm_2_(3) Tb_BSP.Ixx_g_cm_2_(2) nan nan Tb_BSP.Ixx_g_cm_2_(1) Tb_BSP.Ixx_g_cm_2_(8) Tb_BSP.Ixx_g_cm_2_(7)]*1e-7;
IzzG = [Tb_BSP.Iyy_g_cm_2_(4) Tb_BSP.Iyy_g_cm_2_(4) Tb_BSP.Iyy_g_cm_2_(6) Tb_BSP.Iyy_g_cm_2_(6) Tb_BSP.Iyy_g_cm_2_(5) Tb_BSP.Iyy_g_cm_2_(2) Tb_BSP.Iyy_g_cm_2_(5) Tb_BSP.Iyy_g_cm_2_(3) Tb_BSP.Iyy_g_cm_2_(10)+Tb_BSP.Iyy_g_cm_2_(9) Tb_BSP.Iyy_g_cm_2_(1) Tb_BSP.Iyy_g_cm_2_(3) Tb_BSP.Iyy_g_cm_2_(2) nan nan Tb_BSP.Iyy_g_cm_2_(1) Tb_BSP.Iyy_g_cm_2_(8) Tb_BSP.Iyy_g_cm_2_(7)]*1e-7;

xxi = IxxG;
yyi = IyyG + mi.*((l_list-xg_list).^2);
zzi = IzzG + mi.*((l_list-xg_list).^2);
