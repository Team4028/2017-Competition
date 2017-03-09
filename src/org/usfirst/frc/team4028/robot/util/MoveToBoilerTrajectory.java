package org.usfirst.frc.team4028.robot.util;

public class MoveToBoilerTrajectory {
	// This class contains arrays which hold motion profiles for the drivetrain
	public static final int kNumPoints = 121;		
	// Position (rotations)	Velocity (R/S)	Acceleration (R/S^2) Heading () Duration (ms)
	
	public static double [][]LeftPoints = new double [][]{
		{0.000000, 0.000000, 0.000000, 0.000000, 0.02},
		{0.000823, 0.041129, 2.055167, 0.000000, 0.02},
		{0.003224, 0.119966, 3.939380, 0.000002, 0.02},
		{0.007220, 0.199658, 3.982097, 0.000010, 0.02},
		{0.012802, 0.278924, 3.960809, 0.000030, 0.02},
		{0.019936, 0.357492, 3.936756, 0.000074, 0.02},
		{0.028653, 0.435329, 3.887410, 0.000152, 0.02},
		{0.038889, 0.512309, 3.852564, 0.000281, 0.02},
		{0.050648, 0.588179, 3.795145, 0.000476, 0.02},
		{0.063905, 0.662897, 3.736081, 0.000759, 0.02},
		{0.078637, 0.736334, 3.670669, 0.001151, 0.02},
		{0.094815, 0.808376, 3.599632, 0.001675, 0.02},
		{0.112392, 0.878888, 3.525853, 0.002356, 0.02},
		{0.131337, 0.947765, 3.445652, 0.003220, 0.02},
		{0.151644, 1.015007, 3.361036, 0.004296, 0.02},
		{0.173240, 1.080559, 3.279804, 0.005608, 0.02},
		{0.196140, 1.144418, 3.191392, 0.007190, 0.02},
		{0.220274, 1.206614, 3.109596, 0.009065, 0.02},
		{0.245616, 1.267145, 3.026595, 0.011262, 0.02},
		{0.272125, 1.326076, 2.948003, 0.013806, 0.02},
		{0.299799, 1.383519, 2.871754, 0.016725, 0.02},
		{0.328582, 1.439593, 2.804536, 0.020038, 0.02},
		{0.358482, 1.494442, 2.741485, 0.023769, 0.02},
		{0.389451, 1.548241, 2.689586, 0.027934, 0.02},
		{0.421467, 1.601153, 2.646170, 0.032547, 0.02},
		{0.454532, 1.653398, 2.612474, 0.037621, 0.02},
		{0.488636, 1.705216, 2.590902, 0.043164, 0.02},
		{0.523774, 1.756842, 2.581251, 0.049180, 0.02},
		{0.559946, 1.808523, 2.583950, 0.055669, 0.02},
		{0.597159, 1.860511, 2.599199, 0.062628, 0.02},
		{0.635408, 1.913046, 2.627579, 0.070046, 0.02},
		{0.674749, 1.966389, 2.666238, 0.077920, 0.02},
		{0.715149, 2.020764, 2.719770, 0.086225, 0.02},
		{0.756693, 2.076394, 2.780407, 0.094954, 0.02},
		{0.799358, 2.133484, 2.854829, 0.104076, 0.02},
		{0.843198, 2.192190, 2.935533, 0.113572, 0.02},
		{0.888254, 2.252682, 3.024462, 0.123417, 0.02},
		{0.934184, 2.296826, 2.207493, 0.133500, 0.02},
		{0.980198, 2.300166, 0.166954, 0.143608, 0.02},
		{1.026290, 2.304523, 0.217864, 0.153707, 0.02},
		{1.072507, 2.311420, 0.344908, 0.163775, 0.02},
		{1.118911, 2.320549, 0.456518, 0.173799, 0.02},
		{1.165562, 2.331639, 0.554318, 0.183767, 0.02},
		{1.212437, 2.344439, 0.640191, 0.193654, 0.02},
		{1.259607, 2.358723, 0.714227, 0.203457, 0.02},
		{1.307102, 2.374294, 0.778427, 0.213166, 0.02},
		{1.354929, 2.390972, 0.833734, 0.222769, 0.02},
		{1.403094, 2.408587, 0.880907, 0.232254, 0.02},
		{1.451638, 2.426996, 0.920357, 0.241620, 0.02},
		{1.500562, 2.446070, 0.953662, 0.250859, 0.02},
		{1.549863, 2.465687, 0.981104, 0.259962, 0.02},
		{1.599581, 2.485746, 1.002892, 0.268931, 0.02},
		{1.649709, 2.506159, 1.020516, 0.277759, 0.02},
		{1.700242, 2.526838, 1.034042, 0.286441, 0.02},
		{1.751192, 2.547712, 1.043794, 0.294976, 0.02},
		{1.802573, 2.568723, 1.050417, 0.303362, 0.02},
		{1.854373, 2.589816, 1.054572, 0.311596, 0.02},
		{1.906581, 2.610940, 1.056421, 0.319674, 0.02},
		{1.959230, 2.632061, 1.055902, 0.327600, 0.02},
		{2.012284, 2.653147, 1.054472, 0.335366, 0.02},
		{2.065775, 2.674170, 1.051016, 0.342977, 0.02},
		{2.119688, 2.695115, 1.047017, 0.350430, 0.02},
		{2.173986, 2.715956, 1.042475, 0.357719, 0.02},
		{2.228723, 2.736687, 1.036493, 0.364851, 0.02},
		{2.283882, 2.757310, 1.030913, 0.371824, 0.02},
		{2.339425, 2.777814, 1.025414, 0.378631, 0.02},
		{2.395406, 2.798202, 1.019129, 0.385279, 0.02},
		{2.451760, 2.818481, 1.014185, 0.391760, 0.02},
		{2.508543, 2.838656, 1.008569, 0.398079, 0.02},
		{2.565714, 2.858742, 1.004392, 0.404231, 0.02},
		{2.623280, 2.878748, 1.000477, 0.410216, 0.02},
		{2.681272, 2.898697, 0.997138, 0.416035, 0.02},
		{2.739626, 2.918603, 0.995573, 0.421681, 0.02},
		{2.798397, 2.938483, 0.994004, 0.427158, 0.02},
		{2.857569, 2.958368, 0.994183, 0.432461, 0.02},
		{2.917126, 2.978276, 0.995529, 0.437588, 0.02},
		{2.977101, 2.998234, 0.997738, 0.442539, 0.02},
		{3.037479, 3.018271, 1.001617, 0.447309, 0.02},
		{3.098244, 3.038407, 1.006871, 0.451895, 0.02},
		{3.159408, 3.058668, 1.013214, 0.456294, 0.02},
		{3.220510, 3.055613, -0.152795, 0.460471, 0.02},
		{3.280397, 2.993845, -3.087905, 0.464356, 0.02},
		{3.338981, 2.928552, -3.263896, 0.467956, 0.02},
		{3.396230, 2.862523, -3.301571, 0.471283, 0.02},
		{3.452134, 2.795789, -3.337379, 0.474349, 0.02},
		{3.506714, 2.728345, -3.371353, 0.477169, 0.02},
		{3.559917, 2.660213, -3.406687, 0.479753, 0.02},
		{3.611741, 2.591426, -3.439674, 0.482113, 0.02},
		{3.662185, 2.521984, -3.471842, 0.484263, 0.02},
		{3.711224, 2.451899, -3.504123, 0.486214, 0.02},
		{3.758836, 2.381203, -3.535737, 0.487977, 0.02},
		{3.805046, 2.309888, -3.564764, 0.489565, 0.02},
		{3.849808, 2.237966, -3.595865, 0.490990, 0.02},
		{3.893100, 2.165490, -3.625350, 0.492260, 0.02},
		{3.934949, 2.092452, -3.651900, 0.493390, 0.02},
		{3.975333, 2.018846, -3.679617, 0.494390, 0.02},
		{4.014231, 1.944709, -3.706490, 0.495269, 0.02},
		{4.051620, 1.870085, -3.732477, 0.496038, 0.02},
		{4.087527, 1.794971, -3.754904, 0.496707, 0.02},
		{4.121930, 1.719363, -3.778671, 0.497285, 0.02},
		{4.154781, 1.643342, -3.802896, 0.497782, 0.02},
		{4.186130, 1.566910, -3.820252, 0.498205, 0.02},
		{4.215930, 1.490071, -3.842195, 0.498563, 0.02},
		{4.244180, 1.412892, -3.860035, 0.498863, 0.02},
		{4.270905, 1.335343, -3.874717, 0.499112, 0.02},
		{4.296056, 1.257468, -3.893513, 0.499317, 0.02},
		{4.319633, 1.179348, -3.907758, 0.499483, 0.02},
		{4.341658, 1.100956, -3.918497, 0.499617, 0.02},
		{4.362106, 1.022301, -3.932251, 0.499722, 0.02},
		{4.380977, 0.943438, -3.942839, 0.499803, 0.02},
		{4.398268, 0.864377, -3.952291, 0.499865, 0.02},
		{4.413953, 0.785194, -3.963919, 0.499911, 0.02},
		{4.428080, 0.705836, -3.964945, 0.499944, 0.02},
		{4.440598, 0.626324, -3.978279, 0.499966, 0.02},
		{4.451555, 0.546637, -3.975490, 0.499981, 0.02},
		{4.460875, 0.466901, -3.994435, 0.499991, 0.02},
		{4.468632, 0.387096, -3.982731, 0.499996, 0.02},
		{4.474774, 0.307156, -3.997761, 0.499999, 0.02},
		{4.479325, 0.227169, -3.992361, 0.500000, 0.02},
		{4.482260, 0.147279, -4.008770, 0.500000, 0.02},
		{4.483628, 0.065934, -3.920170, 0.500000, 0.02}};
		
	public static double [][]RightPoints = new double[][]{
		{0.000000, 0.000000, 0.000000, 0.000000, 0.02},
		{0.000823, 0.041145, 2.055944, 0.000000, 0.02},
		{0.003229, 0.120184, 3.949508, 0.000002, 0.02},
		{0.007243, 0.200592, 4.017894, 0.000010, 0.02},
		{0.012875, 0.281427, 4.039164, 0.000030, 0.02},
		{0.020114, 0.362740, 4.074328, 0.000074, 0.02},
		{0.029021, 0.444824, 4.099510, 0.000152, 0.02},
		{0.039568, 0.527861, 4.155717, 0.000281, 0.02},
		{0.051801, 0.611882, 4.202881, 0.000476, 0.02},
		{0.065743, 0.697126, 4.262347, 0.000759, 0.02},
		{0.081422, 0.783712, 4.327898, 0.001151, 0.02},
		{0.098869, 0.871750, 4.398910, 0.001675, 0.02},
		{0.118093, 0.961287, 4.477188, 0.002356, 0.02},
		{0.139130, 1.052362, 4.556089, 0.003220, 0.02},
		{0.162039, 1.145104, 4.635649, 0.004296, 0.02},
		{0.186813, 1.239524, 4.724172, 0.005608, 0.02},
		{0.213539, 1.335649, 4.803947, 0.007190, 0.02},
		{0.242210, 1.433497, 4.892100, 0.009065, 0.02},
		{0.272869, 1.532970, 4.973679, 0.011262, 0.02},
		{0.305533, 1.633999, 5.053925, 0.013806, 0.02},
		{0.340269, 1.736527, 5.125717, 0.016725, 0.02},
		{0.377066, 1.840439, 5.197168, 0.020038, 0.02},
		{0.415992, 1.945595, 5.255851, 0.023769, 0.02},
		{0.457035, 2.051835, 5.311314, 0.027934, 0.02},
		{0.500203, 2.158915, 5.355210, 0.032547, 0.02},
		{0.545533, 2.266645, 5.386890, 0.037621, 0.02},
		{0.593029, 2.374820, 5.408787, 0.043164, 0.02},
		{0.642693, 2.483193, 5.418586, 0.049180, 0.02},
		{0.694526, 2.591515, 5.415895, 0.055669, 0.02},
		{0.748521, 2.699535, 5.400549, 0.062628, 0.02},
		{0.804643, 2.806980, 5.373924, 0.070046, 0.02},
		{0.862935, 2.913638, 5.331115, 0.077920, 0.02},
		{0.923297, 3.019257, 5.282965, 0.086225, 0.02},
		{0.985794, 3.123626, 5.216422, 0.094954, 0.02},
		{1.050318, 3.226557, 5.147093, 0.104076, 0.02},
		{1.116870, 3.327833, 5.064202, 0.113572, 0.02},
		{1.185420, 3.427337, 4.974966, 0.123417, 0.02},
		{1.255353, 3.497147, 3.490946, 0.133500, 0.02},
		{1.325369, 3.499985, 0.141889, 0.143608, 0.02},
		{1.395371, 3.499985, -0.000002, 0.153707, 0.02},
		{1.465353, 3.499985, -0.000002, 0.163775, 0.02},
		{1.535343, 3.499985, -0.000009, 0.173799, 0.02},
		{1.605370, 3.499985, 0.000004, 0.183767, 0.02},
		{1.675349, 3.499985, 0.000000, 0.193654, 0.02},
		{1.745342, 3.499985, 0.000021, 0.203457, 0.02},
		{1.815355, 3.499986, 0.000016, 0.213166, 0.02},
		{1.885367, 3.499986, 0.000009, 0.222769, 0.02},
		{1.955355, 3.499986, 0.000016, 0.232254, 0.02},
		{2.025361, 3.499986, 0.000002, 0.241620, 0.02},
		{2.095364, 3.499987, 0.000026, 0.250859, 0.02},
		{2.165346, 3.499987, 0.000027, 0.259962, 0.02},
		{2.235350, 3.499987, 0.000010, 0.268931, 0.02},
		{2.305358, 3.499988, 0.000029, 0.277759, 0.02},
		{2.375352, 3.499988, 0.000010, 0.286441, 0.02},
		{2.445346, 3.499989, 0.000028, 0.294976, 0.02},
		{2.515354, 3.499989, 0.000024, 0.303362, 0.02},
		{2.585359, 3.499990, 0.000013, 0.311596, 0.02},
		{2.655344, 3.499989, -0.000009, 0.319674, 0.02},
		{2.725354, 3.499989, 0.000004, 0.327600, 0.02},
		{2.795342, 3.499990, 0.000034, 0.335366, 0.02},
		{2.865352, 3.499990, 0.000010, 0.342977, 0.02},
		{2.935367, 3.499991, 0.000017, 0.350430, 0.02},
		{3.005339, 3.499991, 0.000019, 0.357719, 0.02},
		{3.075342, 3.499991, 0.000003, 0.364851, 0.02},
		{3.145359, 3.499992, 0.000028, 0.371824, 0.02},
		{3.215342, 3.499992, 0.000013, 0.378631, 0.02},
		{3.285363, 3.499992, 0.000016, 0.385279, 0.02},
		{3.355344, 3.499992, 0.000005, 0.391760, 0.02},
		{3.425356, 3.499992, 0.000002, 0.398079, 0.02},
		{3.495351, 3.499993, 0.000017, 0.404231, 0.02},
		{3.565340, 3.499993, 0.000031, 0.410216, 0.02},
		{3.635362, 3.499993, 0.000008, 0.416035, 0.02},
		{3.705339, 3.499993, -0.000006, 0.421681, 0.02},
		{3.775341, 3.499993, -0.000001, 0.427158, 0.02},
		{3.845346, 3.499993, 0.000007, 0.432461, 0.02},
		{3.915336, 3.499993, -0.000006, 0.437588, 0.02},
		{3.985348, 3.499993, 0.000003, 0.442539, 0.02},
		{4.055362, 3.499993, -0.000029, 0.447309, 0.02},
		{4.125359, 3.499993, -0.000003, 0.451895, 0.02},
		{4.195348, 3.499993, -0.000010, 0.456294, 0.02},
		{4.264804, 3.473403, -1.329721, 0.460471, 0.02},
		{4.332436, 3.381046, -4.617107, 0.464356, 0.02},
		{4.398178, 3.286308, -4.735789, 0.467956, 0.02},
		{4.462022, 3.192322, -4.699484, 0.471283, 0.02},
		{4.523991, 3.099076, -4.663245, 0.474349, 0.02},
		{4.584136, 3.006516, -4.626837, 0.477169, 0.02},
		{4.642427, 2.914631, -4.594368, 0.479753, 0.02},
		{4.698890, 2.823428, -4.560579, 0.482113, 0.02},
		{4.753552, 2.732872, -4.527442, 0.484263, 0.02},
		{4.806412, 2.642948, -4.496063, 0.486214, 0.02},
		{4.857472, 2.553663, -4.465408, 0.487977, 0.02},
		{4.906785, 2.464978, -4.433077, 0.489565, 0.02},
		{4.954326, 2.376873, -4.404965, 0.490990, 0.02},
		{5.000094, 2.289378, -4.376549, 0.492260, 0.02},
		{5.044143, 2.202450, -4.346427, 0.493390, 0.02},
		{5.086472, 2.116041, -4.319632, 0.494390, 0.02},
		{5.127079, 2.030156, -4.293869, 0.495269, 0.02},
		{5.165961, 1.944800, -4.269236, 0.496038, 0.02},
		{5.203168, 1.859925, -4.242847, 0.496707, 0.02},
		{5.238694, 1.775479, -4.220375, 0.497285, 0.02},
		{5.272507, 1.691502, -4.200873, 0.497782, 0.02},
		{5.304678, 1.607945, -4.176359, 0.498205, 0.02},
		{5.335171, 1.524761, -4.159492, 0.498563, 0.02},
		{5.364002, 1.441968, -4.140802, 0.498863, 0.02},
		{5.391211, 1.359483, -4.121391, 0.499112, 0.02},
		{5.416759, 1.277298, -4.108977, 0.499317, 0.02},
		{5.440657, 1.195448, -4.094334, 0.499483, 0.02},
		{5.462940, 1.113854, -4.078559, 0.499617, 0.02},
		{5.483592, 1.032477, -4.068384, 0.499722, 0.02},
		{5.502620, 0.951324, -4.057290, 0.499803, 0.02},
		{5.520031, 0.870362, -4.047283, 0.499865, 0.02},
		{5.535804, 0.789626, -4.041670, 0.499911, 0.02},
		{5.549995, 0.709021, -4.027296, 0.499944, 0.02},
		{5.562557, 0.628527, -4.027364, 0.499966, 0.02},
		{5.573544, 0.548090, -4.012925, 0.499981, 0.02},
		{5.582882, 0.467801, -4.022151, 0.499991, 0.02},
		{5.590649, 0.387607, -4.002129, 0.499996, 0.02},
		{5.596796, 0.307412, -4.010534, 0.499999, 0.02},
		{5.601349, 0.227274, -3.999903, 0.500000, 0.02},
		{5.604285, 0.147309, -4.012526, 0.500000, 0.02},
		{5.605653, 0.065937, -3.921432, 0.500000, 0.02}};
}