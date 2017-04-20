package org.usfirst.frc.team4028.robot.util;

public class MoveToHopperBlue_X {
	// This class contains arrays which hold motion profiles for the drivetrain
	public static final int kNumPoints = 128;		
	// Position (rotations)	Velocity (R/S)	Acceleration (R/S^2) Heading () Duration (ms)
	
	public static double [][]LeftPoints = new double [][]{
		{0.000000, 0.000000, 0.000000, 0.000000, 0.02},
		{0.001025, 0.051250, 2.562501, 0.000100, 0.02},
		{0.004025, 0.150000, 4.937497, 0.000100, 0.02},
		{0.009025, 0.250002, 4.999966, 0.000100, 0.02},
		{0.016026, 0.350009, 4.999872, 0.000100, 0.02},
		{0.025029, 0.450029, 4.999677, 0.000100, 0.02},
		{0.036037, 0.550073, 4.999348, 0.000100, 0.02},
		{0.049029, 0.650072, 5.003325, 0.000100, 0.02},
		{0.064040, 0.750072, 4.997047, 0.000099, 0.02},
		{0.081030, 0.850075, 5.003472, 0.000099, 0.02},
		{0.100042, 0.950078, 4.997323, 0.000099, 0.02},
		{0.121049, 1.050126, 5.001302, 0.000098, 0.02},
		{0.144031, 1.150104, 5.003279, 0.000097, 0.02},
		{0.169052, 1.250109, 4.996590, 0.000096, 0.02},
		{0.196056, 1.350160, 5.002377, 0.000095, 0.02},
		{0.225049, 1.450157, 5.001688, 0.000094, 0.02},
		{0.256044, 1.550141, 5.000456, 0.000092, 0.02},
		{0.289038, 1.650131, 5.000766, 0.000090, 0.02},
		{0.324062, 1.750162, 4.998595, 0.000088, 0.02},
		{0.361048, 1.850183, 5.003400, 0.000085, 0.02},
		{0.400067, 1.950192, 4.998550, 0.000083, 0.02},
		{0.441057, 2.050207, 5.002401, 0.000080, 0.02},
		{0.484070, 2.150212, 4.999209, 0.000077, 0.02},
		{0.529074, 2.250231, 5.001053, 0.000074, 0.02},
		{0.576061, 2.350220, 5.001264, 0.000070, 0.02},
		{0.625081, 2.450223, 4.998643, 0.000067, 0.02},
		{0.676087, 2.550244, 5.000889, 0.000063, 0.02},
		{0.729089, 2.650245, 5.000385, 0.000060, 0.02},
		{0.784085, 2.750236, 5.000268, 0.000056, 0.02},
		{0.841101, 2.850238, 4.999127, 0.000053, 0.02},
		{0.900087, 2.950231, 5.001215, 0.000049, 0.02},
		{0.961116, 3.050232, 4.998081, 0.000046, 0.02},
		{1.024118, 3.150246, 5.000920, 0.000042, 0.02},
		{1.089116, 3.250234, 4.999952, 0.000039, 0.02},
		{1.156124, 3.350226, 4.999363, 0.000036, 0.02},
		{1.225111, 3.450211, 5.000449, 0.000033, 0.02},
		{1.296114, 3.550192, 4.999144, 0.000030, 0.02},
		{1.369123, 3.650189, 4.999466, 0.000027, 0.02},
		{1.444123, 3.750184, 5.000028, 0.000024, 0.02},
		{1.521128, 3.850176, 4.999494, 0.000021, 0.02},
		{1.600149, 3.950182, 4.999176, 0.000018, 0.02},
		{1.681149, 4.050185, 5.000345, 0.000016, 0.02},
		{1.764170, 4.150187, 4.999078, 0.000013, 0.02},
		{1.849162, 4.250184, 5.000558, 0.000011, 0.02},
		{1.936158, 4.350167, 4.999591, 0.000008, 0.02},
		{2.025140, 4.450145, 5.000044, 0.000006, 0.02},
		{2.116182, 4.550150, 4.998080, 0.000004, 0.02},
		{2.209167, 4.650156, 5.001298, 0.000002, 0.02},
		{2.304158, 4.750135, 4.999552, 0.000000, 0.02},
		{2.401171, 4.850129, 4.999190, -0.000002, 0.02},
		{2.500166, 4.950125, 5.000172, -0.000003, 0.02},
		{2.601152, 5.050109, 5.000003, -0.000005, 0.02},
		{2.704181, 5.150109, 4.998722, -0.000006, 0.02},
		{2.809201, 5.250126, 4.999992, -0.000008, 0.02},
		{2.916207, 5.350132, 5.000132, -0.000009, 0.02},
		{3.025188, 5.450119, 5.000374, -0.000011, 0.02},
		{3.136179, 5.550100, 4.999548, -0.000012, 0.02},
		{3.249211, 5.650105, 4.998875, -0.000013, 0.02},
		{3.364206, 5.750111, 5.000657, -0.000015, 0.02},
		{3.481180, 5.850092, 5.000234, -0.000016, 0.02},
		{3.600199, 5.950084, 4.998856, -0.000017, 0.02},
		{3.720188, 6.000000, 2.496033, -0.000018, 0.02},
		{3.840212, 6.000000, 0.000000, -0.000019, 0.02},
		{3.960196, 6.000000, -0.000000, -0.000020, 0.02},
		{4.080176, 6.000000, 0.000000, -0.000020, 0.02},
		{4.200188, 6.000000, -0.000000, -0.000021, 0.02},
		{4.320212, 6.000000, -0.000000, -0.000022, 0.02},
		{4.440175, 6.000000, 0.000000, -0.000022, 0.02},
		{4.560056, 5.993528, -0.323580, -0.000023, 0.02},
		{4.678335, 5.914018, -3.975526, -0.000023, 0.02},
		{4.794640, 5.814005, -4.999601, -0.000024, 0.02},
		{4.908880, 5.714008, -5.001590, -0.000024, 0.02},
		{5.021181, 5.614014, -4.998803, -0.000024, 0.02},
		{5.131466, 5.513999, -5.000512, -0.000025, 0.02},
		{5.239766, 5.413985, -4.999762, -0.000025, 0.02},
		{5.346018, 5.313986, -5.001279, -0.000025, 0.02},
		{5.450313, 5.213989, -4.999076, -0.000025, 0.02},
		{5.552598, 5.113976, -5.000375, -0.000025, 0.02},
		{5.652873, 5.013974, -5.000373, -0.000025, 0.02},
		{5.751142, 4.913979, -5.000268, -0.000025, 0.02},
		{5.847413, 4.813987, -5.000058, -0.000025, 0.02},
		{5.941698, 4.713986, -4.999741, -0.000025, 0.02},
		{6.033966, 4.613988, -5.000582, -0.000025, 0.02},
		{6.124281, 4.513972, -4.998789, -0.000024, 0.02},
		{6.212530, 4.413968, -5.001968, -0.000024, 0.02},
		{6.298829, 4.313972, -4.998662, -0.000024, 0.02},
		{6.383119, 4.213951, -5.000338, -0.000023, 0.02},
		{6.465392, 4.113946, -5.000614, -0.000023, 0.02},
		{6.545645, 4.013964, -5.000751, -0.000022, 0.02},
		{6.623920, 3.913982, -4.999417, -0.000022, 0.02},
		{6.700222, 3.813968, -4.999200, -0.000021, 0.02},
		{6.774484, 3.713963, -5.001439, -0.000021, 0.02},
		{6.846763, 3.613973, -4.999509, -0.000020, 0.02},
		{6.917045, 3.513970, -4.999986, -0.000019, 0.02},
		{6.985323, 3.413968, -5.000213, -0.000018, 0.02},
		{7.051599, 3.313970, -5.000162, -0.000018, 0.02},
		{7.115881, 3.213969, -4.999798, -0.000017, 0.02},
		{7.178187, 3.113944, -4.999086, -0.000016, 0.02},
		{7.238441, 3.013942, -5.002182, -0.000015, 0.02},
		{7.296743, 2.913943, -4.997954, -0.000014, 0.02},
		{7.353006, 2.813936, -5.001730, -0.000013, 0.02},
		{7.407280, 2.713956, -4.999482, -0.000012, 0.02},
		{7.459561, 2.613959, -4.999657, -0.000011, 0.02},
		{7.509851, 2.513948, -4.999448, -0.000010, 0.02},
		{7.558131, 2.413937, -5.000397, -0.000009, 0.02},
		{7.604417, 2.313931, -4.999534, -0.000008, 0.02},
		{7.648673, 2.213953, -5.001580, -0.000007, 0.02},
		{7.690977, 2.113954, -4.997007, -0.000006, 0.02},
		{7.731241, 2.013946, -5.002157, -0.000005, 0.02},
		{7.769517, 1.913974, -4.999112, -0.000004, 0.02},
		{7.805797, 1.813984, -4.999524, -0.000004, 0.02},
		{7.840095, 1.713962, -4.998364, -0.000003, 0.02},
		{7.872367, 1.613949, -5.001576, -0.000002, 0.02},
		{7.902643, 1.513969, -4.999542, -0.000002, 0.02},
		{7.930916, 1.413992, -5.000089, -0.000001, 0.02},
		{7.957197, 1.314009, -4.999132, -0.000001, 0.02},
		{7.981486, 1.213994, -4.998817, -0.000001, 0.02},
		{8.003777, 1.113952, -4.999398, -0.000001, 0.02},
		{8.024055, 1.013929, -5.001264, -0.000000, 0.02},
		{8.042322, 0.913967, -5.001425, -0.000000, 0.02},
		{8.058600, 0.814012, -4.998656, -0.000000, 0.02},
		{8.072902, 0.713939, -4.995303, -0.000000, 0.02},
		{8.085165, 0.613920, -5.007211, -0.000000, 0.02},
		{8.095446, 0.513980, -4.996609, -0.000000, 0.02},
		{8.103723, 0.413994, -5.001077, -0.000000, 0.02},
		{8.110023, 0.313821, -4.989524, -0.000000, 0.02},
		{8.114298, 0.213592, -5.007672, -0.000000, 0.02},
		{8.116573, 0.113400, -4.994184, -0.000000, 0.02}};
	
	public static double [][]RightPoints = new double [][]{
		{0.000000, 0.000000, 0.000000, 0.000000, 0.02},
		{0.001025, 0.051250, 2.562499, 0.000100, 0.02},
		{0.004025, 0.150000, 4.937484, 0.000100, 0.02},
		{0.009025, 0.250001, 4.999922, 0.000100, 0.02},
		{0.016026, 0.350006, 4.999774, 0.000100, 0.02},
		{0.025029, 0.450023, 4.999508, 0.000100, 0.02},
		{0.036036, 0.550062, 4.999088, 0.000100, 0.02},
		{0.049028, 0.650052, 5.002955, 0.000100, 0.02},
		{0.064038, 0.750043, 4.996554, 0.000099, 0.02},
		{0.081028, 0.850034, 5.002846, 0.000099, 0.02},
		{0.100039, 0.950021, 4.996553, 0.000099, 0.02},
		{0.121045, 1.050051, 5.000388, 0.000098, 0.02},
		{0.144025, 1.150008, 5.002223, 0.000097, 0.02},
		{0.169042, 1.249989, 4.995404, 0.000096, 0.02},
		{0.196044, 1.350014, 5.001076, 0.000095, 0.02},
		{0.225033, 1.449983, 5.000293, 0.000094, 0.02},
		{0.256024, 1.549938, 4.998999, 0.000092, 0.02},
		{0.289013, 1.649898, 4.999279, 0.000090, 0.02},
		{0.324032, 1.749900, 4.997113, 0.000088, 0.02},
		{0.361013, 1.849892, 5.001960, 0.000085, 0.02},
		{0.400025, 1.949874, 4.997189, 0.000083, 0.02},
		{0.441009, 2.049863, 5.001149, 0.000080, 0.02},
		{0.484014, 2.149846, 4.998092, 0.000077, 0.02},
		{0.529011, 2.249846, 5.000091, 0.000074, 0.02},
		{0.575990, 2.349819, 5.000470, 0.000070, 0.02},
		{0.625001, 2.449810, 4.998025, 0.000067, 0.02},
		{0.675999, 2.549822, 5.000443, 0.000063, 0.02},
		{0.728992, 2.649818, 5.000108, 0.000060, 0.02},
		{0.783980, 2.749806, 5.000148, 0.000056, 0.02},
		{0.840987, 2.849809, 4.999150, 0.000053, 0.02},
		{0.899965, 2.949804, 5.001365, 0.000049, 0.02},
		{0.960985, 3.049811, 4.998344, 0.000046, 0.02},
		{1.023979, 3.149832, 5.001275, 0.000042, 0.02},
		{1.088968, 3.249829, 5.000385, 0.000039, 0.02},
		{1.155968, 3.349830, 4.999857, 0.000036, 0.02},
		{1.224948, 3.449826, 5.000993, 0.000033, 0.02},
		{1.295943, 3.549819, 4.999722, 0.000030, 0.02},
		{1.368945, 3.649828, 5.000070, 0.000027, 0.02},
		{1.443938, 3.749835, 5.000647, 0.000024, 0.02},
		{1.520936, 3.849840, 5.000121, 0.000021, 0.02},
		{1.599951, 3.949858, 4.999803, 0.000018, 0.02},
		{1.680945, 4.049874, 5.000970, 0.000016, 0.02},
		{1.763960, 4.149888, 4.999695, 0.000013, 0.02},
		{1.848946, 4.249897, 5.001163, 0.000011, 0.02},
		{1.935936, 4.349892, 5.000183, 0.000008, 0.02},
		{2.024914, 4.449882, 5.000621, 0.000006, 0.02},
		{2.115951, 4.549898, 4.998639, 0.000004, 0.02},
		{2.208931, 4.649915, 5.001841, 0.000002, 0.02},
		{2.303917, 4.749904, 5.000077, 0.000000, 0.02},
		{2.400925, 4.849908, 4.999699, -0.000002, 0.02},
		{2.499917, 4.949914, 5.000663, -0.000003, 0.02},
		{2.600898, 5.049908, 5.000477, -0.000005, 0.02},
		{2.703923, 5.149917, 4.999180, -0.000006, 0.02},
		{2.808940, 5.249942, 5.000435, -0.000008, 0.02},
		{2.915942, 5.349957, 5.000560, -0.000009, 0.02},
		{3.024919, 5.449953, 5.000790, -0.000011, 0.02},
		{3.135907, 5.549942, 4.999951, -0.000012, 0.02},
		{3.248937, 5.649954, 4.999267, -0.000013, 0.02},
		{3.363928, 5.749968, 5.001039, -0.000015, 0.02},
		{3.480900, 5.849957, 5.000607, -0.000016, 0.02},
		{3.599917, 5.949956, 4.999222, -0.000017, 0.02},
		{3.719903, 5.999880, 2.496442, -0.000018, 0.02},
		{3.839924, 5.999889, 0.000438, -0.000019, 0.02},
		{3.959907, 5.999897, 0.000416, -0.000020, 0.02},
		{4.079885, 5.999905, 0.000397, -0.000020, 0.02},
		{4.199895, 5.999913, 0.000381, -0.000021, 0.02},
		{4.319918, 5.999920, 0.000368, -0.000022, 0.02},
		{4.439879, 5.999927, 0.000357, -0.000022, 0.02},
		{4.559759, 5.993462, -0.323227, -0.000023, 0.02},
		{4.678037, 5.913960, -3.975145, -0.000023, 0.02},
		{4.794341, 5.813954, -4.999227, -0.000024, 0.02},
		{4.908580, 5.713965, -5.001233, -0.000024, 0.02},
		{5.020880, 5.613977, -4.998460, -0.000024, 0.02},
		{5.131164, 5.513969, -5.000181, -0.000025, 0.02},
		{5.239464, 5.413961, -4.999440, -0.000025, 0.02},
		{5.345715, 5.313968, -5.000965, -0.000025, 0.02},
		{5.450011, 5.213978, -4.998769, -0.000025, 0.02},
		{5.552296, 5.113971, -5.000073, -0.000025, 0.02},
		{5.652570, 5.013974, -5.000075, -0.000025, 0.02},
		{5.750839, 4.913986, -4.999972, -0.000025, 0.02},
		{5.847110, 4.813999, -4.999763, -0.000025, 0.02},
		{5.941396, 4.714005, -4.999447, -0.000025, 0.02},
		{6.033664, 4.614012, -5.000288, -0.000025, 0.02},
		{6.123980, 4.514002, -4.998494, -0.000024, 0.02},
		{6.212230, 4.414004, -5.001672, -0.000024, 0.02},
		{6.298530, 4.314014, -4.998364, -0.000024, 0.02},
		{6.382821, 4.213999, -5.000038, -0.000023, 0.02},
		{6.465095, 4.114000, -5.000313, -0.000023, 0.02},
		{6.545349, 4.014025, -5.000448, -0.000022, 0.02},
		{6.623625, 3.914048, -4.999114, -0.000022, 0.02},
		{6.699929, 3.814040, -4.998899, -0.000021, 0.02},
		{6.774192, 3.714041, -5.001139, -0.000021, 0.02},
		{6.846473, 3.614057, -4.999214, -0.000020, 0.02},
		{6.916756, 3.514060, -4.999696, -0.000019, 0.02},
		{6.985036, 3.414063, -4.999933, -0.000018, 0.02},
		{7.051314, 3.314071, -4.999895, -0.000018, 0.02},
		{7.115599, 3.214075, -4.999549, -0.000017, 0.02},
		{7.177907, 3.114054, -4.998859, -0.000016, 0.02},
		{7.238163, 3.014057, -5.001981, -0.000015, 0.02},
		{7.296468, 2.914060, -4.997787, -0.000014, 0.02},
		{7.352733, 2.814057, -5.001600, -0.000013, 0.02},
		{7.407010, 2.714078, -4.999397, -0.000012, 0.02},
		{7.459293, 2.614082, -4.999622, -0.000011, 0.02},
		{7.509586, 2.514070, -4.999468, -0.000010, 0.02},
		{7.557868, 2.414058, -5.000475, -0.000009, 0.02},
		{7.604156, 2.314049, -4.999674, -0.000008, 0.02},
		{7.648414, 2.214067, -5.001780, -0.000007, 0.02},
		{7.690720, 2.114063, -4.997266, -0.000006, 0.02},
		{7.730987, 2.014049, -5.002470, -0.000005, 0.02},
		{7.769264, 1.914069, -4.999473, -0.000004, 0.02},
		{7.805546, 1.814071, -4.999925, -0.000004, 0.02},
		{7.839845, 1.714040, -4.998793, -0.000003, 0.02},
		{7.872120, 1.614019, -5.002022, -0.000002, 0.02},
		{7.902397, 1.514030, -4.999992, -0.000002, 0.02},
		{7.930671, 1.414044, -5.000532, -0.000001, 0.02},
		{7.956952, 1.314053, -4.999555, -0.000001, 0.02},
		{7.981242, 1.214030, -4.999213, -0.000001, 0.02},
		{8.003534, 1.113980, -4.999758, -0.000001, 0.02},
		{8.023812, 1.013951, -5.001584, -0.000000, 0.02},
		{8.042080, 0.913984, -5.001701, -0.000000, 0.02},
		{8.058357, 0.814024, -4.998886, -0.000000, 0.02},
		{8.072660, 0.713948, -4.995489, -0.000000, 0.02},
		{8.084923, 0.613926, -5.007356, -0.000000, 0.02},
		{8.095204, 0.513983, -4.996715, -0.000000, 0.02},
		{8.103481, 0.413996, -5.001150, -0.000000, 0.02},
		{8.109781, 0.313822, -4.989569, -0.000000, 0.02},
		{8.114056, 0.213592, -5.007697, -0.000000, 0.02},
		{8.116331, 0.113400, -4.994193, -0.000000, 0.02}};
}
