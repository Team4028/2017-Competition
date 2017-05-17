package org.usfirst.frc.team4028.robot.util;

public class MoveToHopperBlue_X {
	// This class contains arrays which hold motion profiles for the drivetrain
	public static final int kNumPoints = 174;		
	// Position (rotations)	Velocity (R/S)	Acceleration (R/S^2) Heading () Duration (ms)
	
	public static double [][]LeftPoints = new double [][]{
		{0.000000, 0.000000, 0.000000, 0.000000, 0.02},
		{0.002025, 0.101250, 5.062501, 0.000100, 0.02},
		{0.008025, 0.300001, 9.937477, 0.000100, 0.02},
		{0.018026, 0.500010, 9.999814, 0.000100, 0.02},
		{0.032030, 0.700041, 9.999347, 0.000100, 0.02},
		{0.050043, 0.900127, 9.998421, 0.000100, 0.02},
		{0.072028, 1.100114, 10.007326, 0.000099, 0.02},
		{0.098031, 1.300054, 9.996134, 0.000099, 0.02},
		{0.128038, 1.500093, 10.000340, 0.000098, 0.02},
		{0.162049, 1.700152, 10.000471, 0.000097, 0.02},
		{0.200036, 1.900155, 10.004467, 0.000096, 0.02},
		{0.242041, 2.100142, 9.998755, 0.000094, 0.02},
		{0.288057, 2.300199, 10.000360, 0.000092, 0.02},
		{0.338046, 2.500222, 10.004115, 0.000089, 0.02},
		{0.392049, 2.700218, 10.000109, 0.000086, 0.02},
		{0.450072, 2.900273, 9.999706, 0.000083, 0.02},
		{0.512059, 3.100299, 10.004387, 0.000078, 0.02},
		{0.578070, 3.300299, 9.999283, 0.000074, 0.02},
		{0.648088, 3.500343, 10.000525, 0.000069, 0.02},
		{0.722095, 3.700376, 10.001698, 0.000064, 0.02},
		{0.800090, 3.900371, 10.001433, 0.000059, 0.02},
		{0.882085, 4.100348, 10.000304, 0.000054, 0.02},
		{0.968105, 4.300353, 9.998677, 0.000048, 0.02},
		{1.058125, 4.500383, 10.000197, 0.000043, 0.02},
		{1.152117, 4.700377, 10.001307, 0.000037, 0.02},
		{1.250126, 4.900358, 9.998821, 0.000032, 0.02},
		{1.352137, 5.100356, 9.999537, 0.000027, 0.02},
		{1.458139, 5.300345, 9.999969, 0.000022, 0.02},
		{1.568161, 5.500342, 9.998513, 0.000017, 0.02},
		{1.682135, 5.700316, 10.001466, 0.000012, 0.02},
		{1.800147, 5.900281, 9.997721, 0.000008, 0.02},
		{1.922148, 6.100269, 9.999785, 0.000004, 0.02},
		{2.048175, 6.300269, 9.998264, -0.000000, 0.02},
		{2.178166, 6.500262, 10.000732, -0.000004, 0.02},
		{2.312186, 6.700248, 9.998187, -0.000008, 0.02},
		{2.450160, 6.900224, 10.000995, -0.000011, 0.02},
		{2.592183, 7.100201, 9.997530, -0.000014, 0.02},
		{2.738205, 7.300213, 9.999386, -0.000017, 0.02},
		{2.888183, 7.500194, 10.000771, -0.000019, 0.02},
		{3.042206, 7.700175, 9.997809, -0.000022, 0.02},
		{3.200202, 7.900170, 10.000166, -0.000024, 0.02},
		{3.362181, 8.100136, 9.999823, -0.000026, 0.02},
		{3.528183, 8.300107, 9.998503, -0.000028, 0.02},
		{3.698194, 8.500097, 9.999002, -0.000029, 0.02},
		{3.872210, 8.700094, 9.999056, -0.000030, 0.02},
		{4.050186, 8.900071, 10.000235, -0.000031, 0.02},
		{4.232185, 9.100038, 9.998424, -0.000032, 0.02},
		{4.417591, 9.268569, 8.425011, -0.000032, 0.02},
		{4.600410, 9.141363, -6.360593, -0.000032, 0.02},
		{4.779239, 8.941345, -10.000795, -0.000031, 0.02},
		{4.954063, 8.741324, -10.001219, -0.000031, 0.02},
		{5.124881, 8.541308, -10.001269, -0.000029, 0.02},
		{5.291707, 8.341290, -10.000874, -0.000028, 0.02},
		{5.454542, 8.141261, -10.000876, -0.000026, 0.02},
		{5.613354, 7.941240, -10.001841, -0.000023, 0.02},
		{5.768171, 7.741231, -10.000916, -0.000020, 0.02},
		{5.919016, 7.541202, -10.000062, -0.000017, 0.02},
		{6.065836, 7.341177, -10.001542, -0.000014, 0.02},
		{6.208663, 7.141171, -10.000010, -0.000010, 0.02},
		{6.347487, 6.941176, -9.999735, -0.000007, 0.02},
		{6.482302, 6.741208, -9.999022, -0.000003, 0.02},
		{6.613125, 6.541265, -9.997388, -0.000001, 0.02},
		{6.739963, 6.341328, -9.995940, -0.000000, 0.02},
		{6.849609, 5.483013, -42.921025, 0.010881, 0.02},
		{6.933514, 4.195426, -64.382024, 0.042494, 0.02},
		{7.026191, 4.633597, 21.907509, 0.070206, 0.02},
		{7.125249, 4.953110, 15.976206, 0.096030, 0.02},
		{7.229156, 5.194584, 12.071985, 0.121291, 0.02},
		{7.336969, 5.391220, 9.832863, 0.146755, 0.02},
		{7.448135, 5.557601, 8.317994, 0.172986, 0.02},
		{7.562127, 5.700201, 7.130724, 0.200424, 0.02},
		{7.676171, 5.702313, 0.105596, 0.228894, 0.02},
		{7.787693, 5.575596, -6.335221, 0.258138, 0.02},
		{7.896166, 5.424175, -7.571800, 0.288311, 0.02},
		{8.001311, 5.256370, -8.388887, 0.319583, 0.02},
		{8.102861, 5.078411, -8.899561, 0.352089, 0.02},
		{8.200769, 4.895050, -9.167375, 0.386005, 0.02},
		{8.294985, 4.709979, -9.251920, 0.421495, 0.02},
		{8.385496, 4.526221, -9.189259, 0.458718, 0.02},
		{8.472429, 4.346191, -9.000523, 0.497891, 0.02},
		{8.555869, 4.171892, -8.714766, 0.539212, 0.02},
		{8.635974, 4.005068, -8.340795, 0.582923, 0.02},
		{8.712912, 3.847217, -7.893242, 0.629282, 0.02},
		{8.786902, 3.699653, -7.378445, 0.678596, 0.02},
		{8.858182, 3.563545, -6.804596, 0.731215, 0.02},
		{8.926976, 3.439992, -6.178195, 0.787509, 0.02},
		{8.993566, 3.330003, -5.500213, 0.847937, 0.02},
		{9.058265, 3.234483, -4.775332, 0.913045, 0.02},
		{9.121360, 3.154311, -4.008051, 0.983425, 0.02},
		{9.183163, 3.090337, -3.198890, 1.059781, 0.02},
		{9.244031, 3.043348, -2.349418, 1.142977, 0.02},
		{9.304314, 3.014103, -1.462230, 1.234017, 0.02},
		{9.364380, 3.003351, -0.537582, 1.334104, 0.02},
		{9.424614, 3.011838, 0.424342, 1.444694, 0.02},
		{9.485413, 3.040324, 1.424465, 1.567553, 0.02},
		{9.547213, 3.089626, 2.464818, 1.704903, 0.02},
		{9.610417, 3.160643, 3.551325, 1.859372, 0.02},
		{9.675513, 3.254417, 4.688116, 2.034429, 0.02},
		{9.742958, 3.372222, 5.890229, 2.234300, 0.02},
		{9.813276, 3.515644, 7.170565, 2.464526, 0.02},
		{9.887014, 3.686768, 8.555920, 2.732252, 0.02},
		{9.964622, 3.881028, 9.714583, 3.046257, 0.02},
		{10.043217, 3.929313, 2.413981, 3.401863, 0.02},
		{10.121286, 3.903967, -1.267482, 3.797098, 0.02},
		{10.198704, 3.870203, -1.687891, 4.235399, 0.02},
		{10.275229, 3.826961, -2.162511, 4.719321, 0.02},
		{10.350710, 3.773691, -2.663270, 5.251412, 0.02},
		{10.424931, 3.710202, -3.173719, 5.832845, 0.02},
		{10.497645, 3.636605, -3.680723, 6.463003, 0.02},
		{10.568697, 3.553166, -4.172670, 7.139982, 0.02},
		{10.637908, 3.460216, -4.646973, 7.859264, 0.02},
		{10.705084, 3.358169, -5.101481, 8.613135, 0.02},
		{10.770044, 3.247472, -5.533928, 9.390821, 0.02},
		{10.832585, 3.128607, -5.946170, 10.178073, 0.02},
		{10.892655, 3.001935, -6.330334, 10.959931, 0.02},
		{10.949999, 2.867863, -6.705141, 11.717237, 0.02},
		{11.004557, 2.726795, -7.050509, 12.433119, 0.02},
		{11.056104, 2.579124, -7.388641, 13.089962, 0.02},
		{11.104614, 2.425186, -7.695856, 13.675730, 0.02},
		{11.149937, 2.265217, -7.995160, 14.181017, 0.02},
		{11.191935, 2.099642, -8.277812, 14.601589, 0.02},
		{11.230529, 1.928801, -8.537978, 14.938811, 0.02},
		{11.265557, 1.753160, -8.790886, 15.197696, 0.02},
		{11.297015, 1.573053, -9.006382, 15.387782, 0.02},
		{11.324811, 1.388657, -9.212007, 15.519869, 0.02},
		{11.348817, 1.200575, -9.406325, 15.605534, 0.02},
		{11.369014, 1.009276, -9.559791, 15.656593, 0.02},
		{11.385343, 0.815010, -9.695698, 15.683598, 0.02},
		{11.397709, 0.618581, -9.826136, 15.695482, 0.02},
		{11.406127, 0.420513, -9.895093, 15.699336, 0.02},
		{11.410572, 0.220353, -9.922603, 15.699985, 0.02}};
	
	public static double [][]RightPoints = new double [][]{
		{0.000000, 0.000000, 0.000000, 0.000000, 0.02},
		{0.002025, 0.101250, 5.062499, 0.000100, 0.02},
		{0.008025, 0.300001, 9.937438, 0.000100, 0.02},
		{0.018026, 0.500006, 9.999676, 0.000100, 0.02},
		{0.032029, 0.700031, 9.999049, 0.000100, 0.02},
		{0.050042, 0.900107, 9.997900, 0.000100, 0.02},
		{0.072026, 1.100078, 10.006534, 0.000099, 0.02},
		{0.098028, 1.299996, 9.995030, 0.000099, 0.02},
		{0.128033, 1.500007, 9.998897, 0.000098, 0.02},
		{0.162043, 1.700030, 9.998684, 0.000097, 0.02},
		{0.200026, 1.899990, 10.002347, 0.000096, 0.02},
		{0.242027, 2.099929, 9.996341, 0.000094, 0.02},
		{0.288037, 2.299933, 9.997717, 0.000092, 0.02},
		{0.338020, 2.499900, 10.001322, 0.000089, 0.02},
		{0.392015, 2.699839, 9.997270, 0.000086, 0.02},
		{0.450030, 2.899839, 9.996921, 0.000083, 0.02},
		{0.512007, 3.099812, 10.001766, 0.000078, 0.02},
		{0.578007, 3.299765, 9.996917, 0.000074, 0.02},
		{0.648014, 3.499768, 9.998493, 0.000069, 0.02},
		{0.722009, 3.699768, 10.000048, 0.000064, 0.02},
		{0.799991, 3.899738, 10.000195, 0.000059, 0.02},
		{0.881973, 4.099699, 9.999479, 0.000054, 0.02},
		{0.967980, 4.299696, 9.998250, 0.000048, 0.02},
		{1.057986, 4.499724, 10.000134, 0.000043, 0.02},
		{1.151966, 4.699723, 10.001568, 0.000037, 0.02},
		{1.249962, 4.899715, 9.999360, 0.000032, 0.02},
		{1.351960, 5.099728, 10.000306, 0.000027, 0.02},
		{1.457950, 5.299737, 10.000921, 0.000022, 0.02},
		{1.567960, 5.499756, 9.999609, 0.000017, 0.02},
		{1.681923, 5.699754, 10.002667, 0.000012, 0.02},
		{1.799924, 5.899744, 9.998997, 0.000008, 0.02},
		{1.921915, 6.099759, 10.001111, 0.000004, 0.02},
		{2.047932, 6.299786, 9.999622, -0.000000, 0.02},
		{2.177914, 6.499806, 10.002105, -0.000004, 0.02},
		{2.311926, 6.699820, 9.999567, -0.000008, 0.02},
		{2.449892, 6.899823, 10.002375, -0.000011, 0.02},
		{2.591907, 7.099828, 9.998905, -0.000014, 0.02},
		{2.737922, 7.299867, 10.000759, -0.000017, 0.02},
		{2.887894, 7.499875, 10.002143, -0.000019, 0.02},
		{3.041911, 7.699885, 9.999188, -0.000022, 0.02},
		{3.199902, 7.899907, 10.001558, -0.000024, 0.02},
		{3.361876, 8.099902, 10.001242, -0.000026, 0.02},
		{3.527874, 8.299902, 9.999961, -0.000028, 0.02},
		{3.697882, 8.499922, 10.000519, -0.000029, 0.02},
		{3.871894, 8.699951, 10.000657, -0.000030, 0.02},
		{4.049869, 8.899963, 10.001947, -0.000031, 0.02},
		{4.231867, 9.099966, 10.000285, -0.000032, 0.02},
		{4.417272, 9.268539, 8.427069, -0.000032, 0.02},
		{4.600091, 9.141378, -6.358366, -0.000032, 0.02},
		{4.778921, 8.941404, -9.998544, -0.000031, 0.02},
		{4.953747, 8.741429, -9.998937, -0.000031, 0.02},
		{5.124568, 8.541460, -9.998935, -0.000029, 0.02},
		{5.291398, 8.341489, -9.998482, -0.000028, 0.02},
		{5.454238, 8.141509, -9.998437, -0.000026, 0.02},
		{5.613056, 7.941537, -9.999398, -0.000023, 0.02},
		{5.767880, 7.741575, -9.998560, -0.000020, 0.02},
		{5.918733, 7.541588, -9.997954, -0.000017, 0.02},
		{6.065561, 7.341595, -9.999939, -0.000014, 0.02},
		{6.208397, 7.141604, -9.999286, -0.000010, 0.02},
		{6.347229, 6.941596, -10.000368, -0.000007, 0.02},
		{6.482052, 6.741578, -10.001519, -0.000003, 0.02},
		{6.612880, 6.541541, -10.002110, -0.000001, 0.02},
		{6.739720, 6.341466, -10.002852, -0.000000, 0.02},
		{6.875699, 6.799769, 22.917965, 0.010881, 0.02},
		{7.036042, 8.017486, 60.888352, 0.042494, 0.02},
		{7.195564, 7.975704, -2.088953, 0.070206, 0.02},
		{7.356683, 8.056238, 4.026829, 0.096030, 0.02},
		{7.521003, 8.214793, 7.926614, 0.121291, 0.02},
		{7.689347, 8.418153, 10.169088, 0.146755, 0.02},
		{7.862404, 8.651748, 11.678281, 0.172986, 0.02},
		{8.040556, 8.908496, 12.838725, 0.200424, 0.02},
		{8.220456, 8.995207, 4.335638, 0.228894, 0.02},
		{8.398781, 8.915442, -3.987911, 0.258138, 0.02},
		{8.575196, 8.821555, -4.694761, 0.288311, 0.02},
		{8.749612, 8.719396, -5.107120, 0.319583, 0.02},
		{8.921847, 8.613295, -5.305996, 0.352089, 0.02},
		{9.091988, 8.506474, -5.340690, 0.386005, 0.02},
		{9.260045, 8.401338, -5.255873, 0.421495, 0.02},
		{9.426015, 8.299710, -5.082167, 0.458718, 0.02},
		{9.590091, 8.202911, -4.839383, 0.497891, 0.02},
		{9.752332, 8.111897, -4.550634, 0.539212, 0.02},
		{9.912887, 8.027362, -4.226524, 0.582923, 0.02},
		{10.071869, 7.949768, -3.880044, 0.629282, 0.02},
		{10.229451, 7.879398, -3.518653, 0.678596, 0.02},
		{10.385798, 7.816396, -3.149683, 0.731215, 0.02},
		{10.541001, 7.760830, -2.778550, 0.787509, 0.02},
		{10.695233, 7.712691, -2.407286, 0.847937, 0.02},
		{10.848693, 7.671918, -2.038351, 0.913045, 0.02},
		{11.001483, 7.638451, -1.673124, 0.983425, 0.02},
		{11.153719, 7.612247, -1.310309, 1.059781, 0.02},
		{11.305586, 7.593287, -0.947996, 1.142977, 0.02},
		{11.457221, 7.581612, -0.583745, 1.234017, 0.02},
		{11.608766, 7.577348, -0.213192, 1.334104, 0.02},
		{11.760373, 7.580725, 0.168879, 1.444694, 0.02},
		{11.912196, 7.592103, 0.568983, 1.567553, 0.02},
		{12.064456, 7.612003, 0.994848, 1.704903, 0.02},
		{12.217256, 7.641132, 1.456685, 1.859372, 0.02},
		{12.370884, 7.680431, 1.964660, 2.034429, 0.02},
		{12.525508, 7.731128, 2.534836, 2.234300, 0.02},
		{12.681415, 7.794806, 3.183670, 2.464526, 0.02},
		{12.838891, 7.873526, 3.935875, 2.732252, 0.02},
		{12.997971, 7.955253, 4.087030, 3.046257, 0.02},
		{13.152272, 7.714233, -12.049697, 3.401863, 0.02},
		{13.299046, 7.339680, -18.730234, 3.797098, 0.02},
		{13.438542, 6.973513, -18.304939, 4.235399, 0.02},
		{13.570853, 6.616820, -17.838044, 4.719321, 0.02},
		{13.696268, 6.270161, -17.331349, 5.251412, 0.02},
		{13.814968, 5.933627, -16.822875, 5.832845, 0.02},
		{13.927086, 5.607257, -16.322360, 6.463003, 0.02},
		{14.032884, 5.290803, -15.825432, 7.139982, 0.02},
		{14.132570, 4.983784, -15.349309, 7.859264, 0.02},
		{14.226303, 4.685789, -14.897116, 8.613135, 0.02},
		{14.314246, 4.396427, -14.465701, 9.390821, 0.02},
		{14.396513, 4.115365, -14.059982, 10.178073, 0.02},
		{14.473393, 3.842037, -13.659365, 10.959931, 0.02},
		{14.544898, 3.576055, -13.302176, 11.717237, 0.02},
		{14.611267, 3.317090, -12.942968, 12.433119, 0.02},
		{14.672521, 3.064818, -12.622227, 13.089962, 0.02},
		{14.728906, 2.818867, -12.295946, 13.675730, 0.02},
		{14.780502, 2.578727, -12.002025, 14.181017, 0.02},
		{14.827391, 2.344197, -11.725176, 14.601589, 0.02},
		{14.869709, 2.114920, -11.458392, 14.938811, 0.02},
		{14.907485, 1.890667, -11.223916, 15.197696, 0.02},
		{14.940901, 1.670997, -10.984760, 15.387782, 0.02},
		{14.970030, 1.455246, -10.778449, 15.519869, 0.02},
		{14.994889, 1.243207, -10.604475, 15.605534, 0.02},
		{15.015589, 1.034445, -10.432417, 15.656593, 0.02},
		{15.032184, 0.828240, -10.291627, 15.683598, 0.02},
		{15.044666, 0.624401, -10.196747, 15.695482, 0.02},
		{15.053121, 0.422396, -10.091819, 15.699336, 0.02},
		{15.057572, 0.220668, -10.000340, 15.699985, 0.02}};
}