package org.usfirst.frc.team4028.robot.util;

public class MoveToHopperTrajectory {
	// This class contains arrays which hold motion profiles for the drivetrain
	public static final int kNumPoints = 108;		
	// Position (rotations)	Velocity (R/S)	Acceleration (R/S^2) Heading () Duration (ms)
	
	public static double [][]LeftPoints = new double [][]{
		{0.000000, 0.000000, 0.000000, 0.000000, 0.02},
		{0.001025, 0.051250, 2.562503, 0.001000, 0.02},
		{0.004025, 0.150001, 4.937536, 0.001000, 0.02},
		{0.009025, 0.250004, 5.000122, 0.001000, 0.02},
		{0.016026, 0.350011, 5.000254, 0.001000, 0.02},
		{0.025027, 0.450026, 5.000422, 0.000999, 0.02},
		{0.036029, 0.550052, 5.000615, 0.000999, 0.02},
		{0.049034, 0.650094, 5.000822, 0.000998, 0.02},
		{0.064044, 0.750160, 5.001031, 0.000996, 0.02},
		{0.081037, 0.850190, 5.004710, 0.000994, 0.02},
		{0.100042, 0.950208, 5.000715, 0.000991, 0.02},
		{0.121065, 1.050307, 5.001046, 0.000988, 0.02},
		{0.144063, 1.150394, 5.006495, 0.000982, 0.02},
		{0.169071, 1.250450, 5.003006, 0.000976, 0.02},
		{0.196075, 1.350526, 5.004939, 0.000968, 0.02},
		{0.225091, 1.450622, 5.004172, 0.000959, 0.02},
		{0.256110, 1.550745, 5.005448, 0.000948, 0.02},
		{0.289128, 1.650872, 5.006285, 0.000934, 0.02},
		{0.324143, 1.750993, 5.006774, 0.000919, 0.02},
		{0.361155, 1.851106, 5.006975, 0.000901, 0.02},
		{0.400170, 1.951219, 5.006928, 0.000881, 0.02},
		{0.441220, 2.051376, 5.005142, 0.000859, 0.02},
		{0.484238, 2.151533, 5.009293, 0.000835, 0.02},
		{0.529263, 2.251654, 5.006920, 0.000808, 0.02},
		{0.576311, 2.351800, 5.006041, 0.000778, 0.02},
		{0.625346, 2.451946, 5.007703, 0.000747, 0.02},
		{0.676387, 2.552074, 5.006408, 0.000713, 0.02},
		{0.729428, 2.652194, 5.006342, 0.000677, 0.02},
		{0.784488, 2.752319, 5.004940, 0.000639, 0.02},
		{0.841536, 2.852435, 5.005912, 0.000600, 0.02},
		{0.900562, 2.952507, 5.005564, 0.000559, 0.02},
		{0.961614, 3.052567, 5.002986, 0.000516, 0.02},
		{1.024680, 3.152644, 5.002799, 0.000473, 0.02},
		{1.089720, 3.252696, 5.003701, 0.000429, 0.02},
		{1.156773, 3.352723, 5.001454, 0.000384, 0.02},
		{1.225848, 3.452762, 5.000480, 0.000338, 0.02},
		{1.296893, 3.552780, 5.001667, 0.000293, 0.02},
		{1.369936, 3.652762, 4.999940, 0.000247, 0.02},
		{1.445002, 3.752746, 4.998472, 0.000202, 0.02},
		{1.522053, 3.852724, 4.999141, 0.000157, 0.02},
		{1.601102, 3.952680, 4.998062, 0.000113, 0.02},
		{1.682161, 4.052632, 4.997221, 0.000070, 0.02},
		{1.765231, 4.152586, 4.996608, 0.000027, 0.02},
		{1.850281, 4.252527, 4.997095, -0.000014, 0.02},
		{1.937301, 4.352432, 4.996863, -0.000054, 0.02},
		{2.026367, 4.452339, 4.994253, -0.000092, 0.02},
		{2.117396, 4.552245, 4.996212, -0.000129, 0.02},
		{2.210450, 4.652139, 4.994080, -0.000164, 0.02},
		{2.305493, 4.752036, 4.994720, -0.000198, 0.02},
		{2.402542, 4.851924, 4.993900, -0.000229, 0.02},
		{2.501573, 4.951787, 4.993349, -0.000258, 0.02},
		{2.601573, 4.999998, 2.410574, -0.000285, 0.02},
		{2.701571, 4.999998, 0.000009, -0.000309, 0.02},
		{2.801578, 4.999998, -0.000007, -0.000331, 0.02},
		{2.901573, 4.999999, 0.000059, -0.000350, 0.02},
		{3.001567, 4.999999, -0.000022, -0.000366, 0.02},
		{3.101571, 4.999999, -0.000016, -0.000380, 0.02},
		{3.201563, 4.999998, -0.000011, -0.000391, 0.02},
		{3.301080, 4.975593, -1.220224, -0.000399, 0.02},
		{3.398688, 4.879993, -4.779579, -0.000405, 0.02},
		{3.494269, 4.779846, -5.008167, -0.000409, 0.02},
		{3.587870, 4.679709, -5.006530, -0.000410, 0.02},
		{3.679450, 4.579577, -5.007178, -0.000409, 0.02},
		{3.769041, 4.479457, -5.005957, -0.000405, 0.02},
		{3.856648, 4.379326, -5.005366, -0.000400, 0.02},
		{3.942224, 4.279208, -5.006355, -0.000393, 0.02},
		{4.025789, 4.179121, -5.005412, -0.000384, 0.02},
		{4.107369, 4.079037, -5.004230, -0.000374, 0.02},
		{4.186966, 3.978940, -5.003738, -0.000362, 0.02},
		{4.264528, 3.878860, -5.004955, -0.000349, 0.02},
		{4.340128, 3.778784, -5.002189, -0.000335, 0.02},
		{4.413697, 3.678710, -5.004023, -0.000320, 0.02},
		{4.485259, 3.578670, -5.002803, -0.000304, 0.02},
		{4.554842, 3.478629, -5.001342, -0.000287, 0.02},
		{4.622420, 3.378584, -5.001735, -0.000270, 0.02},
		{4.687974, 3.278570, -5.002042, -0.000252, 0.02},
		{4.751544, 3.178571, -5.000078, -0.000234, 0.02},
		{4.813115, 3.078570, -5.000106, -0.000216, 0.02},
		{4.872703, 2.978562, -4.998929, -0.000198, 0.02},
		{4.930274, 2.878564, -4.999976, -0.000181, 0.02},
		{4.985846, 2.778589, -4.998705, -0.000163, 0.02},
		{5.039414, 2.678626, -4.998584, -0.000147, 0.02},
		{5.090999, 2.578660, -4.997220, -0.000131, 0.02},
		{5.140567, 2.478702, -4.998467, -0.000115, 0.02},
		{5.188138, 2.378765, -4.997246, -0.000101, 0.02},
		{5.233705, 2.278838, -4.997458, -0.000088, 0.02},
		{5.277282, 2.178912, -4.996396, -0.000075, 0.02},
		{5.318884, 2.078949, -4.995372, -0.000064, 0.02},
		{5.358445, 1.979012, -4.999351, -0.000054, 0.02},
		{5.396024, 1.879106, -4.995710, -0.000044, 0.02},
		{5.431625, 1.779150, -4.995225, -0.000036, 0.02},
		{5.465201, 1.679198, -4.998770, -0.000029, 0.02},
		{5.496801, 1.579247, -4.995239, -0.000023, 0.02},
		{5.526370, 1.479309, -4.999825, -0.000018, 0.02},
		{5.553976, 1.379357, -4.994207, -0.000014, 0.02},
		{5.579559, 1.279379, -4.999793, -0.000010, 0.02},
		{5.603156, 1.179412, -4.996459, -0.000007, 0.02},
		{5.624727, 1.079472, -5.001173, -0.000005, 0.02},
		{5.644331, 0.979511, -4.994614, -0.000003, 0.02},
		{5.661922, 0.879500, -5.000134, -0.000002, 0.02},
		{5.677506, 0.779539, -5.000222, -0.000001, 0.02},
		{5.691111, 0.679528, -4.995490, -0.000001, 0.02},
		{5.702688, 0.579551, -5.004939, -0.000000, 0.02},
		{5.712288, 0.479574, -4.994084, -0.000000, 0.02},
		{5.719888, 0.379460, -4.998468, -0.000000, 0.02},
		{5.725488, 0.279259, -4.996789, -0.000000, 0.02},
		{5.729063, 0.179275, -5.013906, -0.000000, 0.02},
		{5.730663, 0.078404, -4.942917, -0.000000, 0.02}};
		
	public static double [][]RightPoints = new double[][]{
		{0.000000, 0.000000, 0.000000, 0.000000, 0.02},
		{0.001025, 0.051250, 2.562497, 0.001000, 0.02},
		{0.004025, 0.149999, 4.937460, 0.001000, 0.02},
		{0.009025, 0.249997, 4.999852, 0.001000, 0.02},
		{0.016025, 0.349993, 4.999663, 0.001000, 0.02},
		{0.025025, 0.449986, 4.999387, 0.000999, 0.02},
		{0.036026, 0.549980, 4.999019, 0.000999, 0.02},
		{0.049029, 0.649977, 4.998553, 0.000998, 0.02},
		{0.064036, 0.749982, 4.997990, 0.000996, 0.02},
		{0.081023, 0.849934, 5.000801, 0.000994, 0.02},
		{0.100021, 0.949855, 4.995871, 0.000991, 0.02},
		{0.121034, 1.049838, 4.995205, 0.000988, 0.02},
		{0.144020, 1.149787, 4.999605, 0.000982, 0.02},
		{0.169013, 1.249683, 4.995061, 0.000976, 0.02},
		{0.195998, 1.349580, 4.995929, 0.000968, 0.02},
		{0.224991, 1.449475, 4.994134, 0.000959, 0.02},
		{0.255983, 1.549377, 4.994428, 0.000948, 0.02},
		{0.288969, 1.649266, 4.994363, 0.000934, 0.02},
		{0.323947, 1.749132, 4.994053, 0.000919, 0.02},
		{0.360916, 1.848978, 4.993583, 0.000901, 0.02},
		{0.399883, 1.948813, 4.993021, 0.000881, 0.02},
		{0.440879, 2.048685, 4.990886, 0.000859, 0.02},
		{0.483838, 2.148553, 4.994859, 0.000835, 0.02},
		{0.528797, 2.248386, 4.992514, 0.000808, 0.02},
		{0.575774, 2.348248, 4.991856, 0.000778, 0.02},
		{0.624732, 2.448118, 4.993923, 0.000747, 0.02},
		{0.675692, 2.547982, 4.993229, 0.000713, 0.02},
		{0.728646, 2.647855, 4.993931, 0.000677, 0.02},
		{0.783615, 2.747750, 4.993460, 0.000639, 0.02},
		{0.840567, 2.847658, 4.995493, 0.000600, 0.02},
		{0.899494, 2.947544, 4.996322, 0.000559, 0.02},
		{0.960443, 3.047445, 4.995013, 0.000516, 0.02},
		{1.023404, 3.147389, 4.996157, 0.000473, 0.02},
		{1.088337, 3.247336, 4.998427, 0.000429, 0.02},
		{1.155282, 3.347286, 4.997573, 0.000384, 0.02},
		{1.224247, 3.447274, 4.997982, 0.000338, 0.02},
		{1.295181, 3.547270, 5.000533, 0.000293, 0.02},
		{1.368114, 3.647255, 5.000138, 0.000247, 0.02},
		{1.443071, 3.747269, 4.999950, 0.000202, 0.02},
		{1.520013, 3.847301, 5.001846, 0.000157, 0.02},
		{1.598956, 3.947335, 5.001932, 0.000113, 0.02},
		{1.679909, 4.047386, 5.002190, 0.000070, 0.02},
		{1.762877, 4.147460, 5.002610, 0.000027, 0.02},
		{1.847827, 4.247541, 5.004072, -0.000014, 0.02},
		{1.934751, 4.347603, 5.004751, -0.000054, 0.02},
		{2.023724, 4.447685, 5.002998, -0.000092, 0.02},
		{2.114663, 4.547783, 5.005772, -0.000129, 0.02},
		{2.207633, 4.647883, 5.004410, -0.000164, 0.02},
		{2.302595, 4.748001, 5.005795, -0.000198, 0.02},
		{2.399567, 4.848126, 5.005695, -0.000229, 0.02},
		{2.498528, 4.948239, 5.005857, -0.000258, 0.02},
		{2.598462, 4.996746, 2.425387, -0.000285, 0.02},
		{2.698402, 4.997073, 0.016361, -0.000309, 0.02},
		{2.798357, 4.997396, 0.016107, -0.000331, 0.02},
		{2.898307, 4.997715, 0.015972, -0.000350, 0.02},
		{2.998261, 4.998029, 0.015728, -0.000366, 0.02},
		{3.098232, 4.998342, 0.015610, -0.000380, 0.02},
		{3.198197, 4.998652, 0.015539, -0.000391, 0.02},
		{3.297693, 4.974561, -1.204505, -0.000399, 0.02},
		{3.395287, 4.879279, -4.763675, -0.000405, 0.02},
		{3.490860, 4.779434, -4.993088, -0.000409, 0.02},
		{3.584458, 4.679581, -4.992295, -0.000410, 0.02},
		{3.676041, 4.579719, -4.993753, -0.000409, 0.02},
		{3.765640, 4.479851, -4.993325, -0.000405, 0.02},
		{3.853259, 4.379957, -4.993515, -0.000400, 0.02},
		{3.938852, 4.280061, -4.995280, -0.000393, 0.02},
		{4.022438, 4.180179, -4.995120, -0.000384, 0.02},
		{4.104043, 4.080285, -4.994736, -0.000374, 0.02},
		{4.183669, 3.980362, -4.995056, -0.000362, 0.02},
		{4.261263, 3.880439, -4.997105, -0.000349, 0.02},
		{4.336897, 3.780502, -4.995203, -0.000335, 0.02},
		{4.410503, 3.680551, -4.997920, -0.000320, 0.02},
		{4.482104, 3.580615, -4.997615, -0.000304, 0.02},
		{4.551727, 3.480659, -4.997093, -0.000287, 0.02},
		{4.619347, 3.380680, -4.998444, -0.000270, 0.02},
		{4.684944, 3.280711, -4.999725, -0.000252, 0.02},
		{4.748557, 3.180740, -4.998741, -0.000234, 0.02},
		{4.810172, 3.080746, -4.999747, -0.000216, 0.02},
		{4.869803, 2.980726, -4.999530, -0.000198, 0.02},
		{4.927417, 2.880697, -5.001512, -0.000181, 0.02},
		{4.983031, 2.780673, -5.001133, -0.000163, 0.02},
		{5.036639, 2.680645, -5.001845, -0.000147, 0.02},
		{5.088262, 2.580598, -5.001246, -0.000131, 0.02},
		{5.137867, 2.480546, -5.003174, -0.000115, 0.02},
		{5.185474, 2.380504, -5.002535, -0.000101, 0.02},
		{5.231073, 2.280461, -5.003228, -0.000088, 0.02},
		{5.274680, 2.180412, -5.002530, -0.000075, 0.02},
		{5.316310, 2.080322, -5.001756, -0.000064, 0.02},
		{5.355895, 1.980254, -5.005871, -0.000054, 0.02},
		{5.393496, 1.880218, -5.002247, -0.000044, 0.02},
		{5.429117, 1.780132, -5.001670, -0.000036, 0.02},
		{5.462711, 1.680055, -5.005030, -0.000029, 0.02},
		{5.494325, 1.579984, -5.001215, -0.000023, 0.02},
		{5.523906, 1.479935, -5.005450, -0.000018, 0.02},
		{5.551523, 1.379878, -4.999408, -0.000014, 0.02},
		{5.577114, 1.279806, -5.004533, -0.000010, 0.02},
		{5.600718, 1.179754, -5.000697, -0.000007, 0.02},
		{5.622295, 1.079739, -5.004896, -0.000005, 0.02},
		{5.641903, 0.979715, -4.997812, -0.000003, 0.02},
		{5.659497, 0.879649, -5.002824, -0.000002, 0.02},
		{5.675083, 0.779645, -5.002420, -0.000001, 0.02},
		{5.688689, 0.679599, -4.997227, -0.000001, 0.02},
		{5.700267, 0.579596, -5.006260, -0.000000, 0.02},
		{5.709868, 0.479600, -4.995033, -0.000000, 0.02},
		{5.717468, 0.379473, -4.999101, -0.000000, 0.02},
		{5.723068, 0.279264, -4.997168, -0.000000, 0.02},
		{5.726643, 0.179276, -5.014095, -0.000000, 0.02},
		{5.728243, 0.078404, -4.942980, -0.000000, 0.02}};
}
