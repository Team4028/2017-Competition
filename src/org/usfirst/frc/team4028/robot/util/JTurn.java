package org.usfirst.frc.team4028.robot.util;

public class JTurn {
	// This class contains arrays which hold motion profiles for the drivetrain
	public static final int kNumPoints = 93;		
	// Position (rotations)	Velocity (R/S)	Acceleration (R/S^2) Heading () Duration (ms)
	
	public static double [][]LeftPoints = new double [][]{
		{0.000000, 0.000000, 0.000000, 0.000000, 0.02},
		{0.000903, 0.045138, 2.256738, 0.000999, 0.02},
		{0.003614, 0.135522, 4.518188, 0.000991, 0.02},
		{0.008154, 0.227027, 4.575922, 0.000958, 0.02},
		{0.014543, 0.319447, 4.620561, 0.000885, 0.02},
		{0.022775, 0.411638, 4.610401, 0.000775, 0.02},
		{0.032824, 0.502498, 4.543144, 0.000651, 0.02},
		{0.044667, 0.592091, 4.479342, 0.000534, 0.02},
		{0.058290, 0.681087, 4.449264, 0.000434, 0.02},
		{0.073690, 0.769999, 4.445732, 0.000352, 0.02},
		{0.090871, 0.859044, 4.452281, 0.000285, 0.02},
		{0.109834, 0.948272, 4.461856, 0.000231, 0.02},
		{0.130588, 1.037668, 4.469653, 0.000187, 0.02},
		{0.153135, 1.127208, 4.476468, 0.000151, 0.02},
		{0.177467, 1.216840, 4.482526, 0.000121, 0.02},
		{0.203600, 1.306548, 4.484977, 0.000095, 0.02},
		{0.231529, 1.396329, 4.488614, 0.000074, 0.02},
		{0.261252, 1.486149, 4.491057, 0.000055, 0.02},
		{0.292771, 1.575998, 4.492549, 0.000039, 0.02},
		{0.326086, 1.665871, 4.493995, 0.000024, 0.02},
		{0.361198, 1.755762, 4.494927, 0.000012, 0.02},
		{0.398112, 1.845673, 4.495508, 0.000001, 0.02},
		{0.436833, 1.935611, 4.495916, -0.000009, 0.02},
		{0.477341, 2.025556, 4.497615, -0.000018, 0.02},
		{0.519646, 2.115495, 4.497383, -0.000026, 0.02},
		{0.563757, 2.205447, 4.497395, -0.000033, 0.02},
		{0.609672, 2.295415, 4.497791, -0.000040, 0.02},
		{0.657376, 2.385383, 4.498697, -0.000046, 0.02},
		{0.706881, 2.475347, 4.498407, -0.000051, 0.02},
		{0.758189, 2.565318, 4.498459, -0.000056, 0.02},
		{0.811287, 2.655288, 4.499116, -0.000061, 0.02},
		{0.866195, 2.745261, 4.498405, -0.000065, 0.02},
		{0.922897, 2.835240, 4.499209, -0.000069, 0.02},
		{0.981402, 2.925218, 4.498829, -0.000073, 0.02},
		{1.041709, 3.015201, 4.498945, -0.000076, 0.02},
		{1.103816, 3.105186, 4.498979, -0.000079, 0.02},
		{1.167713, 3.195163, 4.499295, -0.000081, 0.02},
		{1.231488, 3.188769, -0.319696, -0.000083, 0.02},
		{1.293063, 3.078770, -5.499976, -0.000085, 0.02},
		{1.352443, 2.968728, -5.501599, -0.000086, 0.02},
		{1.409611, 2.858681, -5.502936, -0.000086, 0.02},
		{1.464588, 2.748614, -5.502910, -0.000085, 0.02},
		{1.517356, 2.638493, -5.506183, -0.000082, 0.02},
		{1.567921, 2.528252, -5.512007, -0.000074, 0.02},
		{1.616272, 2.417594, -5.533053, -0.000056, 0.02},
		{1.662384, 2.305682, -5.595858, -0.000006, 0.02},
		{1.697950, 1.778260, -26.370340, 0.006943, 0.02},
		{1.709057, 0.555333, -61.144766, 0.032299, 0.02},
		{1.714407, 0.267479, -14.391690, 0.069529, 0.02},
		{1.729050, 0.732222, 23.238918, 0.112881, 0.02},
		{1.748085, 0.951717, 10.974114, 0.158777, 0.02},
		{1.768912, 1.041437, 4.486448, 0.206101, 0.02},
		{1.790056, 1.057212, 0.788753, 0.254471, 0.02},
		{1.810535, 1.023915, -1.664761, 0.303687, 0.02},
		{1.829646, 0.955516, -3.419932, 0.353650, 0.02},
		{1.846867, 0.860952, -4.727716, 0.404328, 0.02},
		{1.861799, 0.746643, -5.715573, 0.455709, 0.02},
		{1.874151, 0.617599, -6.452043, 0.507822, 0.02},
		{1.883711, 0.477962, -6.981968, 0.560705, 0.02},
		{1.890335, 0.331247, -7.336373, 0.614410, 0.02},
		{1.893943, 0.180399, -7.541547, 0.669023, 0.02},
		{1.894500, 0.027846, -7.628764, 0.724600, 0.02},
		{1.896991, 0.124567, 4.836045, 0.781253, 0.02},
		{1.902503, 0.275588, 7.550713, 0.839076, 0.02},
		{1.910995, 0.424547, 7.447297, 0.898167, 0.02},
		{1.922422, 0.571360, 7.340910, 0.958606, 0.02},
		{1.936752, 0.716549, 7.259865, 1.020476, 0.02},
		{1.953978, 0.861276, 7.236233, 1.083826, 0.02},
		{1.973828, 0.992446, 6.558116, 1.147742, 0.02},
		{1.995122, 1.064783, 3.617037, 1.208993, 0.02},
		{2.017451, 1.116501, 2.586089, 1.267178, 0.02},
		{2.040544, 1.154599, 1.904833, 1.322110, 0.02},
		{2.064170, 1.181257, 1.332810, 1.373567, 0.02},
		{2.088129, 1.197991, 0.836763, 1.421324, 0.02},
		{2.112244, 1.205823, 0.391617, 1.465182, 0.02},
		{2.136354, 1.205400, -0.021164, 1.504984, 0.02},
		{2.160301, 1.197091, -0.415386, 1.540620, 0.02},
		{2.183916, 1.181075, -0.800994, 1.572034, 0.02},
		{2.207070, 1.157397, -1.183600, 1.599296, 0.02},
		{2.229586, 1.126032, -1.568583, 1.622499, 0.02},
		{2.251330, 1.086926, -1.954781, 1.641864, 0.02},
		{2.272125, 1.040046, -2.344668, 1.657642, 0.02},
		{2.291838, 0.985402, -2.731544, 1.670178, 0.02},
		{2.310298, 0.923078, -3.116431, 1.679834, 0.02},
		{2.327368, 0.853264, -3.489749, 1.687018, 0.02},
		{2.342891, 0.776268, -3.850489, 1.692134, 0.02},
		{2.356746, 0.692503, -4.186503, 1.695594, 0.02},
		{2.368790, 0.602543, -4.500709, 1.697775, 0.02},
		{2.378939, 0.507014, -4.772356, 1.699031, 0.02},
		{2.387067, 0.406727, -5.018172, 1.699663, 0.02},
		{2.393127, 0.302508, -5.202508, 1.699921, 0.02},
		{2.397029, 0.195395, -5.363460, 1.699992, 0.02},
		{2.398767, 0.085762, -5.412784, 1.700000, 0.02}};
	
	public static double [][]RightPoints = new double [][]{
		{0.000000, 0.000000, 0.000000, 0.000000, 0.02},
		{0.000901, 0.045069, 2.253260, 0.000999, 0.02},
		{0.003592, 0.134512, 4.471171, 0.000991, 0.02},
		{0.008052, 0.223014, 4.425780, 0.000958, 0.02},
		{0.014264, 0.310589, 4.378314, 0.000885, 0.02},
		{0.022231, 0.398390, 4.390863, 0.000775, 0.02},
		{0.031981, 0.487510, 4.456136, 0.000651, 0.02},
		{0.043540, 0.577921, 4.520275, 0.000534, 0.02},
		{0.056920, 0.668942, 4.550445, 0.000434, 0.02},
		{0.072120, 0.760038, 4.554974, 0.000352, 0.02},
		{0.089140, 0.850990, 4.547589, 0.000285, 0.02},
		{0.107973, 0.941752, 4.538627, 0.000231, 0.02},
		{0.128621, 1.032349, 4.529696, 0.000187, 0.02},
		{0.151080, 1.122823, 4.523142, 0.000151, 0.02},
		{0.175339, 1.213183, 4.518949, 0.000121, 0.02},
		{0.201411, 1.303465, 4.513647, 0.000095, 0.02},
		{0.229287, 1.393702, 4.511421, 0.000074, 0.02},
		{0.258965, 1.483889, 4.509410, 0.000055, 0.02},
		{0.290445, 1.574037, 4.507476, 0.000039, 0.02},
		{0.323725, 1.664155, 4.506271, 0.000024, 0.02},
		{0.358807, 1.754250, 4.505113, 0.000012, 0.02},
		{0.395694, 1.844332, 4.504053, 0.000001, 0.02},
		{0.434391, 1.934415, 4.503135, -0.000009, 0.02},
		{0.474877, 2.024482, 4.503773, -0.000018, 0.02},
		{0.517164, 2.114527, 4.502670, -0.000026, 0.02},
		{0.561257, 2.204570, 4.501975, -0.000033, 0.02},
		{0.607156, 2.294618, 4.501784, -0.000040, 0.02},
		{0.654846, 2.384657, 4.502207, -0.000046, 0.02},
		{0.704337, 2.474683, 4.501511, -0.000051, 0.02},
		{0.755633, 2.564709, 4.501236, -0.000056, 0.02},
		{0.808720, 2.654729, 4.501617, -0.000061, 0.02},
		{0.863618, 2.744748, 4.500692, -0.000065, 0.02},
		{0.920310, 2.834770, 4.501328, -0.000069, 0.02},
		{0.978806, 2.924788, 4.500837, -0.000073, 0.02},
		{1.039106, 3.014810, 4.500902, -0.000076, 0.02},
		{1.101206, 3.104835, 4.500966, -0.000079, 0.02},
		{1.165097, 3.194854, 4.501432, -0.000081, 0.02},
		{1.228867, 3.188517, -0.316875, -0.000083, 0.02},
		{1.290438, 3.078587, -5.496523, -0.000085, 0.02},
		{1.349816, 2.968623, -5.497703, -0.000086, 0.02},
		{1.406983, 2.858675, -5.498010, -0.000086, 0.02},
		{1.461963, 2.748749, -5.495807, -0.000085, 0.02},
		{1.514739, 2.638868, -5.494221, -0.000082, 0.02},
		{1.565321, 2.529117, -5.487485, -0.000074, 0.02},
		{1.613716, 2.419785, -5.466783, -0.000056, 0.02},
		{1.659948, 2.311713, -5.403837, -0.000006, 0.02},
		{1.712332, 2.619119, 15.369862, 0.006943, 0.02},
		{1.784772, 3.621937, 50.139594, 0.032299, 0.02},
		{1.869270, 4.224611, 30.131674, 0.069529, 0.02},
		{1.958650, 4.469318, 12.236264, 0.112881, 0.02},
		{2.048644, 4.499434, 1.505709, 0.158777, 0.02},
		{2.138624, 4.499484, 0.002500, 0.206101, 0.02},
		{2.228614, 4.499489, 0.000270, 0.254471, 0.02},
		{2.318607, 4.499491, 0.000079, 0.303687, 0.02},
		{2.408599, 4.499509, 0.000906, 0.353650, 0.02},
		{2.498598, 4.499513, 0.000224, 0.404328, 0.02},
		{2.588586, 4.499525, 0.000598, 0.455709, 0.02},
		{2.678579, 4.499535, 0.000470, 0.507822, 0.02},
		{2.768569, 4.499552, 0.000870, 0.560705, 0.02},
		{2.858552, 4.499563, 0.000539, 0.614410, 0.02},
		{2.948554, 4.499584, 0.001046, 0.669023, 0.02},
		{3.038533, 4.499592, 0.000434, 0.724600, 0.02},
		{3.128525, 4.499615, 0.001130, 0.781253, 0.02},
		{3.218522, 4.499642, 0.001339, 0.839076, 0.02},
		{3.308523, 4.499676, 0.001700, 0.898167, 0.02},
		{3.398514, 4.499691, 0.000752, 0.958606, 0.02},
		{3.488502, 4.499681, -0.000461, 1.020476, 0.02},
		{3.578498, 4.499703, 0.001063, 1.083826, 0.02},
		{3.667257, 4.437732, -3.098345, 1.147742, 0.02},
		{3.750619, 4.168311, -13.471835, 1.208993, 0.02},
		{3.828547, 3.896637, -13.584447, 1.267178, 0.02},
		{3.901321, 3.638568, -12.903088, 1.322110, 0.02},
		{3.969163, 3.391920, -12.331640, 1.373567, 0.02},
		{4.032264, 3.155201, -11.836597, 1.421324, 0.02},
		{4.090809, 2.927392, -11.390941, 1.465182, 0.02},
		{4.144970, 2.707821, -10.977713, 1.504984, 0.02},
		{4.194901, 2.496108, -10.583649, 1.540620, 0.02},
		{4.240732, 2.292136, -10.201150, 1.572034, 0.02},
		{4.282659, 2.095819, -9.813475, 1.599296, 0.02},
		{4.320795, 1.907182, -9.433736, 1.622499, 0.02},
		{4.355330, 1.726282, -9.042558, 1.641864, 0.02},
		{4.386384, 1.553166, -8.658310, 1.657642, 0.02},
		{4.414147, 1.387816, -8.265577, 1.670178, 0.02},
		{4.438748, 1.230123, -7.885201, 1.679834, 0.02},
		{4.460352, 1.079915, -7.508344, 1.687018, 0.02},
		{4.479087, 0.936901, -7.152009, 1.692134, 0.02},
		{4.495106, 0.800640, -6.810210, 1.695594, 0.02},
		{4.508511, 0.670620, -6.504864, 1.697775, 0.02},
		{4.519442, 0.546120, -6.219686, 1.699031, 0.02},
		{4.527964, 0.426397, -5.990709, 1.699663, 0.02},
		{4.534184, 0.310522, -5.784405, 1.699921, 0.02},
		{4.538131, 0.197616, -5.653491, 1.699992, 0.02},
		{4.539873, 0.086010, -5.510211, 1.700000, 0.02}};
}