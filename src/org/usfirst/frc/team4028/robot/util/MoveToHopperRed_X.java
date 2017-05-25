package org.usfirst.frc.team4028.robot.util;

public class MoveToHopperRed_X {
	// This class contains arrays which hold motion profiles for the drivetrain
	public static final int kNumPoints = 129;		
	// Position (rotations)	Velocity (R/S)	Acceleration (R/S^2) Heading () Duration (ms)
	
	public static double [][]LeftPoints = new double [][]{
		{0.000000, 0.000000, 0.000000, 0.000000, 0.02},
		{0.002050, 0.101865, 5.061729, 0.000100, 0.02},
		{0.008050, 0.300935, 9.984573, 0.000100, 0.02},
		{0.018049, 0.500515, 9.989875, 0.000100, 0.02},
		{0.032046, 0.700337, 9.998092, 0.000100, 0.02},
		{0.050036, 0.900191, 10.000515, 0.000100, 0.02},
		{0.072034, 1.100096, 9.997216, 0.000100, 0.02},
		{0.098046, 1.300119, 9.997237, 0.000100, 0.02},
		{0.128045, 1.500150, 10.002882, 0.000099, 0.02},
		{0.162037, 1.700114, 10.001146, 0.000099, 0.02},
		{0.200036, 1.900084, 9.999375, 0.000099, 0.02},
		{0.242030, 2.100069, 10.000936, 0.000098, 0.02},
		{0.288031, 2.300063, 9.999853, 0.000097, 0.02},
		{0.338038, 2.500087, 10.000138, 0.000096, 0.02},
		{0.392035, 2.700107, 10.001916, 0.000094, 0.02},
		{0.450052, 2.900144, 9.999444, 0.000093, 0.02},
		{0.512057, 3.100197, 10.002375, 0.000090, 0.02},
		{0.578044, 3.300205, 10.003048, 0.000088, 0.02},
		{0.648053, 3.500224, 10.000275, 0.000084, 0.02},
		{0.722064, 3.700285, 10.002401, 0.000080, 0.02},
		{0.800057, 3.900334, 10.004122, 0.000075, 0.02},
		{0.882067, 4.100393, 10.002689, 0.000069, 0.02},
		{0.968075, 4.300487, 10.004915, 0.000061, 0.02},
		{1.058102, 4.500619, 10.005068, 0.000051, 0.02},
		{1.152116, 4.700785, 10.008389, 0.000038, 0.02},
		{1.250132, 4.900971, 10.009712, 0.000023, 0.02},
		{1.352145, 5.101188, 10.011926, 0.000004, 0.02},
		{1.458172, 5.301430, 10.012193, -0.000020, 0.02},
		{1.568207, 5.501635, 10.010039, -0.000047, 0.02},
		{1.682238, 5.701639, 10.000372, -0.000074, 0.02},
		{1.800262, 5.901262, 9.981230, -0.000095, 0.02},
		{1.922154, 6.094149, 9.643618, -0.000103, 0.02},
		{2.042713, 6.028322, -3.291545, -0.000100, 0.02},
		{2.159272, 5.827903, -10.020853, -0.000089, 0.02},
		{2.271826, 5.627767, -10.006928, -0.000076, 0.02},
		{2.380388, 5.427780, -9.998736, -0.000064, 0.02},
		{2.484943, 5.227853, -9.996598, -0.000052, 0.02},
		{2.585508, 5.027941, -9.994956, -0.000043, 0.02},
		{2.682063, 4.828028, -9.996234, -0.000034, 0.02},
		{2.774634, 4.628102, -9.995313, -0.000027, 0.02},
		{2.863195, 4.428162, -9.997295, -0.000022, 0.02},
		{2.947755, 4.228228, -9.997254, -0.000017, 0.02},
		{3.028327, 4.028274, -9.996855, -0.000013, 0.02},
		{3.104899, 3.828299, -9.997928, -0.000010, 0.02},
		{3.177462, 3.628332, -9.998909, -0.000007, 0.02},
		{3.246026, 3.428374, -9.998406, -0.000005, 0.02},
		{3.310602, 3.228392, -9.997719, -0.000003, 0.02},
		{3.371159, 3.028422, -10.000486, -0.000002, 0.02},
		{3.427727, 2.828463, -9.998070, -0.000001, 0.02},
		{3.480296, 2.628484, -9.999169, -0.000000, 0.02},
		{3.528873, 2.428486, -9.998346, -0.000000, 0.02},
		{3.573457, 2.228449, -9.998465, -0.000000, 0.02},
		{3.599846, 1.320062, -45.440867, 0.056594, 0.02},
		{3.605287, 0.272057, -52.401433, 0.089808, 0.02},
		{3.627585, 1.114945, 42.145876, 0.112445, 0.02},
		{3.657847, 1.512912, 19.895917, 0.131870, 0.02},
		{3.693647, 1.790201, 13.866127, 0.150081, 0.02},
		{3.733959, 2.015333, 11.255073, 0.167968, 0.02},
		{3.778183, 2.211204, 9.793618, 0.186029, 0.02},
		{3.825925, 2.387268, 8.803842, 0.204618, 0.02},
		{3.876887, 2.547865, 8.029020, 0.224031, 0.02},
		{3.930780, 2.694886, 7.351634, 0.244530, 0.02},
		{3.987355, 2.828908, 6.701542, 0.266393, 0.02},
		{4.046360, 2.949725, 6.039789, 0.289920, 0.02},
		{4.107483, 3.056489, 5.338792, 0.315433, 0.02},
		{4.170438, 3.147886, 4.570016, 0.343332, 0.02},
		{4.234888, 3.222207, 3.715764, 0.374087, 0.02},
		{4.300438, 3.277343, 2.756665, 0.408266, 0.02},
		{4.366643, 3.310869, 1.676595, 0.446569, 0.02},
		{4.432878, 3.311603, 0.036665, 0.489787, 0.02},
		{4.495728, 3.141946, -8.481354, 0.536698, 0.02},
		{4.554068, 2.917476, -11.225417, 0.586997, 0.02},
		{4.608077, 2.699897, -10.876645, 0.641213, 0.02},
		{4.657916, 2.492305, -10.381191, 0.699879, 0.02},
		{4.703869, 2.297391, -9.744417, 0.763705, 0.02},
		{4.746218, 2.117531, -8.993353, 0.833462, 0.02},
		{4.785317, 1.954948, -8.129168, 0.910125, 0.02},
		{4.821551, 1.811688, -7.163053, 0.994861, 0.02},
		{4.855341, 1.689721, -6.099140, 1.089095, 0.02},
		{4.887163, 1.590911, -4.939883, 1.194647, 0.02},
		{4.917504, 1.517078, -3.691774, 1.313717, 0.02},
		{4.946906, 1.469999, -2.353767, 1.449192, 0.02},
		{4.975928, 1.451421, -0.929092, 1.604704, 0.02},
		{5.005191, 1.463114, 0.584629, 1.785178, 0.02},
		{5.035332, 1.506962, 2.192284, 1.997050, 0.02},
		{5.067040, 1.585085, 3.905316, 2.249190, 0.02},
		{5.101036, 1.700017, 5.747423, 2.553772, 0.02},
		{5.138139, 1.855079, 7.752745, 2.928598, 0.02},
		{5.179236, 2.054936, 9.993268, 3.399681, 0.02},
		{5.225364, 2.306470, 12.577089, 4.007080, 0.02},
		{5.277617, 2.612895, 15.322606, 4.812263, 0.02},
		{5.336908, 2.963928, 17.548021, 5.907199, 0.02},
		{5.404060, 3.357831, 19.696712, 7.433756, 0.02},
		{5.479964, 3.795527, 21.886624, 9.611312, 0.02},
		{5.565564, 4.279730, 24.208680, 12.742598, 0.02},
		{5.661872, 4.815991, 26.816189, 17.067731, 0.02},
		{5.770174, 5.414600, 29.927797, 22.052915, 0.02},
		{5.889643, 5.974126, 27.979437, 24.968633, 0.02},
		{6.011676, 6.101162, 6.351300, 25.000000, 0.02},
		{6.129754, 5.904197, -9.848785, 25.000000, 0.02},
		{6.243845, 5.704198, -9.999337, 25.000000, 0.02},
		{6.353921, 5.504201, -10.000534, 25.000000, 0.02},
		{6.460010, 5.304207, -9.999304, 25.000000, 0.02},
		{6.562093, 5.104207, -10.000047, 25.000000, 0.02},
		{6.660171, 4.904219, -10.000064, 25.000000, 0.02},
		{6.754257, 4.704230, -9.999332, 24.999999, 0.02},
		{6.844340, 4.504238, -9.999734, 25.000000, 0.02},
		{6.930429, 4.304238, -9.999570, 25.000000, 0.02},
		{7.012516, 4.104223, -10.000495, 25.000000, 0.02},
		{7.090596, 3.904218, -10.000710, 25.000000, 0.02},
		{7.164679, 3.704220, -10.000158, 25.000000, 0.02},
		{7.234769, 3.504209, -9.999632, 25.000000, 0.02},
		{7.300859, 3.304186, -10.000210, 25.000000, 0.02},
		{7.362936, 3.104184, -10.001160, 25.000000, 0.02},
		{7.421030, 2.904174, -9.998780, 25.000000, 0.02},
		{7.475104, 2.704173, -10.001855, 25.000000, 0.02},
		{7.525192, 2.504178, -9.998756, 25.000000, 0.02},
		{7.571280, 2.304156, -10.000037, 25.000000, 0.02},
		{7.613353, 2.104168, -10.001827, 25.000000, 0.02},
		{7.651442, 1.904178, -9.998235, 25.000000, 0.02},
		{7.685524, 1.704168, -10.000876, 25.000000, 0.02},
		{7.715606, 1.504176, -10.000123, 25.000000, 0.02},
		{7.741701, 1.304132, -9.997360, 25.000000, 0.02},
		{7.763777, 1.104113, -10.003943, 25.000000, 0.02},
		{7.781871, 0.904069, -9.994844, 25.000000, 0.02},
		{7.795954, 0.703970, -10.002427, 25.000000, 0.02},
		{7.806029, 0.503994, -10.003756, 25.000000, 0.02},
		{7.812109, 0.304074, -9.999003, 25.000000, 0.02},
		{7.814192, 0.102052, -9.898023, 25.000000, 0.02}};
	
	public static double [][]RightPoints = new double [][]{
		{0.000000, 0.000000, 0.000000, 0.000000, 0.02},
		{0.002050, 0.101865, 5.061728, 0.000100, 0.02},
		{0.008050, 0.300934, 9.984560, 0.000100, 0.02},
		{0.018049, 0.500514, 9.989830, 0.000100, 0.02},
		{0.032046, 0.700334, 9.997991, 0.000100, 0.02},
		{0.050036, 0.900185, 10.000342, 0.000100, 0.02},
		{0.072033, 1.100084, 9.996949, 0.000100, 0.02},
		{0.098045, 1.300099, 9.996855, 0.000100, 0.02},
		{0.128044, 1.500120, 10.002371, 0.000099, 0.02},
		{0.162035, 1.700071, 10.000481, 0.000099, 0.02},
		{0.200032, 1.900024, 9.998538, 0.000099, 0.02},
		{0.242025, 2.099988, 9.999903, 0.000098, 0.02},
		{0.288023, 2.299957, 9.998597, 0.000097, 0.02},
		{0.338028, 2.499951, 9.998617, 0.000096, 0.02},
		{0.392022, 2.699935, 10.000092, 0.000094, 0.02},
		{0.450034, 2.899927, 9.997254, 0.000093, 0.02},
		{0.512034, 3.099927, 9.999736, 0.000090, 0.02},
		{0.578014, 3.299872, 9.999862, 0.000088, 0.02},
		{0.648015, 3.499814, 9.996398, 0.000084, 0.02},
		{0.722016, 3.699780, 9.997643, 0.000080, 0.02},
		{0.799997, 3.899710, 9.998228, 0.000075, 0.02},
		{0.881991, 4.099622, 9.995314, 0.000069, 0.02},
		{0.967980, 4.299530, 9.995591, 0.000061, 0.02},
		{1.057982, 4.499424, 9.993195, 0.000051, 0.02},
		{1.151967, 4.699287, 9.993236, 0.000038, 0.02},
		{1.249945, 4.899091, 9.990595, 0.000023, 0.02},
		{1.351911, 5.098844, 9.988729, 0.000004, 0.02},
		{1.457881, 5.298579, 9.986869, -0.000020, 0.02},
		{1.567851, 5.498376, 9.989618, -0.000047, 0.02},
		{1.681817, 5.698373, 10.000025, -0.000074, 0.02},
		{1.799791, 5.898745, 10.018670, -0.000095, 0.02},
		{1.921662, 6.093100, 9.717019, -0.000103, 0.02},
		{2.042229, 6.028771, -3.216671, -0.000100, 0.02},
		{2.158815, 5.829198, -9.978505, -0.000089, 0.02},
		{2.271399, 5.629330, -9.993551, -0.000076, 0.02},
		{2.379993, 5.429303, -10.000693, -0.000064, 0.02},
		{2.484574, 5.229218, -10.004548, -0.000052, 0.02},
		{2.585163, 5.029118, -10.004345, -0.000043, 0.02},
		{2.681738, 4.829025, -10.005226, -0.000034, 0.02},
		{2.774326, 4.628939, -10.003308, -0.000027, 0.02},
		{2.862900, 4.428861, -10.004203, -0.000022, 0.02},
		{2.947472, 4.228809, -10.003162, -0.000017, 0.02},
		{3.028054, 4.028753, -10.001901, -0.000013, 0.02},
		{3.104634, 3.828692, -10.002254, -0.000010, 0.02},
		{3.177203, 3.628651, -10.002631, -0.000007, 0.02},
		{3.245772, 3.428629, -10.001623, -0.000005, 0.02},
		{3.310353, 3.228591, -10.000507, -0.000003, 0.02},
		{3.370912, 3.028572, -10.002910, -0.000002, 0.02},
		{3.427483, 2.828571, -10.000170, -0.000001, 0.02},
		{3.480053, 2.628556, -10.000981, -0.000000, 0.02},
		{3.528631, 2.428527, -9.999892, -0.000000, 0.02},
		{3.573215, 2.228464, -9.999762, -0.000000, 0.02},
		{3.683620, 5.522834, 164.796583, 0.056594, 0.02},
		{3.768999, 4.269086, -62.688742, 0.089808, 0.02},
		{3.845522, 3.826258, -22.142214, 0.112445, 0.02},
		{3.922098, 3.828329, 0.103538, 0.131870, 0.02},
		{4.001109, 3.951045, 6.136587, 0.150081, 0.02},
		{4.083639, 4.125914, 8.742278, 0.167968, 0.02},
		{4.170239, 4.330063, 10.207505, 0.186029, 0.02},
		{4.261312, 4.553974, 11.196381, 0.204618, 0.02},
		{4.357190, 4.793371, 11.968569, 0.224031, 0.02},
		{4.458108, 5.046338, 12.649397, 0.244530, 0.02},
		{4.564347, 5.312265, 13.297222, 0.266393, 0.02},
		{4.676195, 5.591444, 13.956505, 0.289920, 0.02},
		{4.793875, 5.884659, 14.662361, 0.315433, 0.02},
		{4.917734, 6.193189, 15.427107, 0.343332, 0.02},
		{5.048122, 6.518822, 16.280262, 0.374087, 0.02},
		{5.185402, 6.863643, 17.240144, 0.408266, 0.02},
		{5.329976, 7.230003, 18.321289, 0.446569, 0.02},
		{5.481989, 7.600345, 18.516327, 0.489787, 0.02},
		{5.634685, 7.633485, 1.656710, 0.536698, 0.02},
		{5.785537, 7.543851, -4.482487, 0.586997, 0.02},
		{5.934816, 7.462380, -4.072649, 0.641213, 0.02},
		{6.082589, 7.389777, -3.630751, 0.699879, 0.02},
		{6.229134, 7.326299, -3.173449, 0.763705, 0.02},
		{6.374566, 7.271862, -2.721987, 0.833462, 0.02},
		{6.519088, 7.226149, -2.285609, 0.910125, 0.02},
		{6.662860, 7.188674, -1.873809, 0.994861, 0.02},
		{6.806019, 7.158871, -1.490338, 1.089095, 0.02},
		{6.948761, 7.136163, -1.135250, 1.194647, 0.02},
		{7.091157, 7.120044, -0.805987, 1.313717, 0.02},
		{7.233369, 7.110150, -0.494669, 1.449192, 0.02},
		{7.375466, 7.106334, -0.190799, 1.604704, 0.02},
		{7.517647, 7.108750, 0.120764, 1.785178, 0.02},
		{7.660013, 7.117941, 0.459521, 1.997050, 0.02},
		{7.802741, 7.134946, 0.850085, 2.249190, 0.02},
		{7.945950, 7.161437, 1.324718, 2.553772, 0.02},
		{8.089955, 7.199927, 1.924413, 2.928598, 0.02},
		{8.235032, 7.254126, 2.710082, 3.399681, 0.02},
		{8.381616, 7.329463, 3.766959, 4.007080, 0.02},
		{8.529880, 7.413848, 4.219626, 4.812263, 0.02},
		{8.679169, 7.462895, 2.451856, 5.907199, 0.02},
		{8.828539, 7.469097, 0.310148, 7.433756, 0.02},
		{8.977156, 7.431468, -1.881649, 9.611312, 0.02},
		{9.124112, 7.347371, -4.204594, 12.742598, 0.02},
		{9.268320, 7.211214, -6.808650, 17.067731, 0.02},
		{9.408586, 7.012704, -9.924578, 22.052915, 0.02},
		{9.540846, 6.613752, -19.949856, 24.968633, 0.02},
		{9.663001, 6.107232, -25.323883, 25.000000, 0.02},
		{9.781079, 5.904197, -10.152305, 25.000000, 0.02},
		{9.895170, 5.704198, -9.999337, 25.000000, 0.02},
		{10.005246, 5.504201, -10.000537, 25.000000, 0.02},
		{10.111335, 5.304207, -9.999302, 25.000000, 0.02},
		{10.213418, 5.104207, -10.000048, 25.000000, 0.02},
		{10.311496, 4.904219, -10.000060, 25.000000, 0.02},
		{10.405582, 4.704230, -9.999343, 24.999999, 0.02},
		{10.495665, 4.504239, -9.999719, 25.000000, 0.02},
		{10.581754, 4.304238, -9.999575, 25.000000, 0.02},
		{10.663841, 4.104222, -10.000502, 25.000000, 0.02},
		{10.741921, 3.904218, -10.000708, 25.000000, 0.02},
		{10.816004, 3.704220, -10.000154, 25.000000, 0.02},
		{10.886094, 3.504208, -9.999636, 25.000000, 0.02},
		{10.952184, 3.304186, -10.000206, 25.000000, 0.02},
		{11.014261, 3.104184, -10.001162, 25.000000, 0.02},
		{11.072355, 2.904175, -9.998780, 25.000000, 0.02},
		{11.126429, 2.704173, -10.001854, 25.000000, 0.02},
		{11.176517, 2.504178, -9.998756, 25.000000, 0.02},
		{11.222605, 2.304156, -10.000039, 25.000000, 0.02},
		{11.264678, 2.104168, -10.001824, 25.000000, 0.02},
		{11.302767, 1.904178, -9.998237, 25.000000, 0.02},
		{11.336849, 1.704168, -10.000876, 25.000000, 0.02},
		{11.366931, 1.504176, -10.000123, 25.000000, 0.02},
		{11.393026, 1.304132, -9.997358, 25.000000, 0.02},
		{11.415101, 1.104113, -10.003946, 25.000000, 0.02},
		{11.433196, 0.904069, -9.994841, 25.000000, 0.02},
		{11.447279, 0.703970, -10.002430, 25.000000, 0.02},
		{11.457354, 0.503994, -10.003755, 25.000000, 0.02},
		{11.463434, 0.304074, -9.999004, 25.000000, 0.02},
		{11.465517, 0.102052, -9.898021, 25.000000, 0.02}};
}