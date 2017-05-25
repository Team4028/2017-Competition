package org.usfirst.frc.team4028.robot.util;

public class MoveToHopperBlue_X {
	// This class contains arrays which hold motion profiles for the drivetrain
	public static final int kNumPoints = 150;		
	// Position (rotations)	Velocity (R/S)	Acceleration (R/S^2) Heading () Duration (ms)
	
	public static double [][]LeftPoints = new double [][]{
		{0.000000, 0.000000, 0.000000, 0.000000, 0.02},
		{0.002025, 0.101250, 5.062501, 0.000100, 0.02},
		{0.008025, 0.300001, 9.937478, 0.000100, 0.02},
		{0.018026, 0.500009, 9.999820, 0.000100, 0.02},
		{0.032030, 0.700040, 9.999367, 0.000100, 0.02},
		{0.050042, 0.900124, 9.998468, 0.000100, 0.02},
		{0.072026, 1.100106, 10.007416, 0.000099, 0.02},
		{0.098028, 1.300035, 9.996287, 0.000099, 0.02},
		{0.128030, 1.500057, 10.000579, 0.000098, 0.02},
		{0.162034, 1.700085, 10.000817, 0.000097, 0.02},
		{0.200033, 1.900105, 10.001768, 0.000096, 0.02},
		{0.242043, 2.100138, 9.999923, 0.000094, 0.02},
		{0.288053, 2.300194, 10.001548, 0.000092, 0.02},
		{0.338049, 2.500218, 10.002873, 0.000089, 0.02},
		{0.392066, 2.700252, 9.999432, 0.000086, 0.02},
		{0.450056, 2.900276, 10.003904, 0.000083, 0.02},
		{0.512063, 3.100278, 9.999801, 0.000079, 0.02},
		{0.578065, 3.300298, 10.001671, 0.000074, 0.02},
		{0.648068, 3.500308, 10.000911, 0.000069, 0.02},
		{0.722079, 3.700327, 10.000349, 0.000064, 0.02},
		{0.800095, 3.900357, 10.000349, 0.000059, 0.02},
		{0.882100, 4.100374, 10.001176, 0.000054, 0.02},
		{0.968117, 4.300387, 9.999534, 0.000048, 0.02},
		{1.058117, 4.500390, 10.001007, 0.000043, 0.02},
		{1.152108, 4.700362, 10.000384, 0.000037, 0.02},
		{1.250132, 4.900356, 9.997935, 0.000032, 0.02},
		{1.352145, 5.100372, 10.000267, 0.000027, 0.02},
		{1.458135, 5.300351, 10.000618, 0.000022, 0.02},
		{1.568131, 5.500315, 9.999105, 0.000017, 0.02},
		{1.682138, 5.700294, 9.998892, 0.000012, 0.02},
		{1.800170, 5.900304, 9.998305, 0.000008, 0.02},
		{1.922178, 6.100315, 10.000325, 0.000003, 0.02},
		{2.048162, 6.300286, 10.000284, -0.000001, 0.02},
		{2.178172, 6.500258, 9.998295, -0.000004, 0.02},
		{2.312194, 6.700260, 9.998800, -0.000008, 0.02},
		{2.450192, 6.900254, 10.000210, -0.000011, 0.02},
		{2.592178, 7.100222, 9.999690, -0.000014, 0.02},
		{2.738177, 7.300192, 9.998802, -0.000017, 0.02},
		{2.888185, 7.500178, 9.999001, -0.000020, 0.02},
		{3.042203, 7.700175, 9.998975, -0.000022, 0.02},
		{3.200193, 7.900162, 10.000158, -0.000025, 0.02},
		{3.362197, 8.100140, 9.998805, -0.000026, 0.02},
		{3.527795, 8.280461, 9.016686, -0.000028, 0.02},
		{3.691300, 8.174687, -5.288327, -0.000030, 0.02},
		{3.850777, 7.974672, -10.001743, -0.000031, 0.02},
		{4.006276, 7.774662, -10.000132, -0.000031, 0.02},
		{4.157764, 7.574647, -10.001127, -0.000032, 0.02},
		{4.305283, 7.374618, -9.999661, -0.000032, 0.02},
		{4.448785, 7.174579, -10.001175, -0.000032, 0.02},
		{4.588257, 6.974572, -10.001768, -0.000032, 0.02},
		{4.723745, 6.774577, -10.000005, -0.000032, 0.02},
		{4.855247, 6.574559, -10.000055, -0.000031, 0.02},
		{4.982749, 6.374530, -10.000581, -0.000030, 0.02},
		{5.106224, 6.174523, -10.001574, -0.000029, 0.02},
		{5.225711, 5.974528, -10.000050, -0.000028, 0.02},
		{5.341204, 5.774519, -10.000268, -0.000027, 0.02},
		{5.452690, 5.574511, -10.000716, -0.000025, 0.02},
		{5.560190, 5.374497, -9.999764, -0.000023, 0.02},
		{5.663690, 5.174471, -10.000295, -0.000022, 0.02},
		{5.763150, 4.974484, -10.002286, -0.000020, 0.02},
		{5.858658, 4.774492, -9.997788, -0.000017, 0.02},
		{5.950150, 4.574469, -10.000807, -0.000015, 0.02},
		{6.037635, 4.374472, -10.000447, -0.000013, 0.02},
		{6.121118, 4.174489, -9.999886, -0.000011, 0.02},
		{6.200608, 3.974503, -9.999338, -0.000009, 0.02},
		{6.276103, 3.774506, -9.999193, -0.000007, 0.02},
		{6.347589, 3.574518, -10.000079, -0.000005, 0.02},
		{6.415079, 3.374539, -9.998989, -0.000004, 0.02},
		{6.478581, 3.174538, -9.998342, -0.000003, 0.02},
		{6.538057, 2.974562, -10.001283, -0.000002, 0.02},
		{6.593573, 2.774558, -9.995668, -0.000001, 0.02},
		{6.645061, 2.574535, -10.001785, -0.000000, 0.02},
		{6.692542, 2.374578, -10.000128, -0.000000, 0.02},
		{6.732224, 1.984410, -19.511343, 0.003144, 0.02},
		{6.767667, 1.772054, -10.617224, 0.067713, 0.02},
		{6.779628, 0.598101, -58.699797, 0.096443, 0.02},
		{6.804482, 1.242554, 32.219651, 0.117879, 0.02},
		{6.836385, 1.595179, 17.631520, 0.136849, 0.02},
		{6.873478, 1.854666, 12.974535, 0.154909, 0.02},
		{6.914891, 2.070476, 10.789469, 0.172802, 0.02},
		{6.960090, 2.260354, 9.495738, 0.190971, 0.02},
		{7.008729, 2.431945, 8.579538, 0.209755, 0.02},
		{7.060515, 2.588780, 7.840030, 0.229438, 0.02},
		{7.115152, 2.732285, 7.176504, 0.250275, 0.02},
		{7.172413, 2.862799, 6.525105, 0.272558, 0.02},
		{7.232005, 2.979926, 5.856917, 0.296580, 0.02},
		{7.293667, 3.082696, 5.137905, 0.322701, 0.02},
		{7.357051, 3.169681, 4.349925, 0.351315, 0.02},
		{7.421845, 3.239053, 3.467889, 0.382936, 0.02},
		{7.487616, 3.288617, 2.478228, 0.418150, 0.02},
		{7.553923, 3.315876, 1.363143, 0.457711, 0.02},
		{7.619647, 3.285765, -1.505340, 0.502122, 0.02},
		{7.681261, 3.080978, -10.240258, 0.549893, 0.02},
		{7.738420, 2.858044, -11.146998, 0.601197, 0.02},
		{7.791278, 2.642877, -10.758273, 0.656547, 0.02},
		{7.840046, 2.438455, -10.221249, 0.716535, 0.02},
		{7.884993, 2.247375, -9.554290, 0.781866, 0.02},
		{7.926439, 2.071952, -8.769689, 0.853395, 0.02},
		{7.964730, 1.914353, -7.879124, 0.932117, 0.02},
		{8.000254, 1.776649, -6.886885, 1.019241, 0.02},
		{8.033477, 1.660733, -5.794259, 1.116363, 0.02},
		{8.064846, 1.568469, -4.613364, 1.225304, 0.02},
		{8.094876, 1.501684, -3.339696, 1.348470, 0.02},
		{8.124120, 1.462116, -1.978302, 1.488949, 0.02},
		{8.153146, 1.451518, -0.529971, 1.650654, 0.02},
		{8.182584, 1.471686, 1.008271, 1.838893, 0.02},
		{8.213070, 1.524545, 2.643365, 2.060569, 0.02},
		{8.245324, 1.612306, 4.387023, 2.325479, 0.02},
		{8.280077, 1.737695, 6.269510, 2.646997, 0.02},
		{8.318155, 1.904283, 8.331170, 3.044643, 0.02},
		{8.360498, 2.117237, 10.648006, 3.547743, 0.02},
		{8.408191, 2.384042, 13.336835, 4.201575, 0.02},
		{8.462255, 2.703347, 15.966263, 5.074119, 0.02},
		{8.523568, 3.065845, 18.125950, 6.268419, 0.02},
		{8.593001, 3.471400, 20.276369, 7.945336, 0.02},
		{8.671429, 3.921324, 22.495705, 10.347917, 0.02},
		{8.759801, 4.418852, 24.877764, 13.788969, 0.02},
		{8.859199, 4.970621, 27.592531, 18.409396, 0.02},
		{8.970959, 5.588232, 30.881749, 23.217274, 0.02},
		{9.092261, 6.064798, 23.827155, 25.000000, 0.02},
		{9.213276, 6.050295, -0.725087, 25.000000, 0.02},
		{9.330281, 5.850289, -10.000352, 25.000000, 0.02},
		{9.443279, 5.650299, -10.000245, 25.000000, 0.02},
		{9.552287, 5.450307, -9.999400, 25.000000, 0.02},
		{9.657288, 5.250313, -10.000224, 25.000000, 0.02},
		{9.758299, 5.050318, -9.999284, 25.000000, 0.02},
		{9.855305, 4.850318, -9.999991, 25.000000, 0.02},
		{9.948310, 4.650327, -9.999798, 25.000000, 0.02},
		{10.037313, 4.450340, -9.999664, 25.000001, 0.02},
		{10.122320, 4.250343, -9.999942, 25.000000, 0.02},
		{10.203332, 4.050329, -10.000017, 25.000000, 0.02},
		{10.280333, 3.850322, -10.000983, 25.000000, 0.02},
		{10.353346, 3.650314, -9.999464, 25.000000, 0.02},
		{10.422352, 3.450301, -10.000691, 25.000000, 0.02},
		{10.487353, 3.250305, -10.000601, 25.000000, 0.02},
		{10.548369, 3.050293, -9.998937, 25.000000, 0.02},
		{10.605380, 2.850265, -10.000485, 25.000000, 0.02},
		{10.658381, 2.650261, -10.001011, 25.000000, 0.02},
		{10.707383, 2.450273, -10.000007, 25.000000, 0.02},
		{10.752384, 2.250287, -10.000314, 25.000000, 0.02},
		{10.793398, 2.050277, -9.998573, 25.000000, 0.02},
		{10.830397, 1.850271, -10.001837, 25.000000, 0.02},
		{10.863404, 1.650283, -9.999112, 25.000000, 0.02},
		{10.892424, 1.450224, -9.997446, 25.000000, 0.02},
		{10.917430, 1.250157, -10.002240, 25.000000, 0.02},
		{10.938436, 1.050130, -9.999779, 25.000000, 0.02},
		{10.955432, 0.850155, -10.002916, 25.000000, 0.02},
		{10.968449, 0.650084, -9.992255, 25.000000, 0.02},
		{10.977455, 0.449875, -10.000908, 25.000000, 0.02},
		{10.982446, 0.249920, -10.011772, 25.000000, 0.02}};
	
	public static double [][]RightPoints = new double [][]{
		{0.000000, 0.000000, 0.000000, 0.000000, 0.02},
		{0.002025, 0.101250, 5.062499, 0.000100, 0.02},
		{0.008025, 0.300001, 9.937439, 0.000100, 0.02},
		{0.018026, 0.500006, 9.999683, 0.000100, 0.02},
		{0.032029, 0.700030, 9.999072, 0.000100, 0.02},
		{0.050042, 0.900104, 9.997952, 0.000100, 0.02},
		{0.072025, 1.100070, 10.006632, 0.000099, 0.02},
		{0.098025, 1.299978, 9.995194, 0.000099, 0.02},
		{0.128026, 1.499971, 9.999149, 0.000098, 0.02},
		{0.162027, 1.699964, 9.999045, 0.000097, 0.02},
		{0.200023, 1.899941, 9.999665, 0.000096, 0.02},
		{0.242029, 2.099926, 9.997525, 0.000094, 0.02},
		{0.288033, 2.299930, 9.998917, 0.000092, 0.02},
		{0.338023, 2.499898, 10.000088, 0.000089, 0.02},
		{0.392033, 2.699876, 9.996595, 0.000086, 0.02},
		{0.450014, 2.899844, 10.001114, 0.000083, 0.02},
		{0.512011, 3.099793, 9.997166, 0.000079, 0.02},
		{0.578002, 3.299766, 9.999284, 0.000074, 0.02},
		{0.647994, 3.499734, 9.998845, 0.000069, 0.02},
		{0.721993, 3.699719, 9.998661, 0.000064, 0.02},
		{0.799997, 3.899724, 9.999069, 0.000059, 0.02},
		{0.881988, 4.099724, 10.000307, 0.000054, 0.02},
		{0.967992, 4.299727, 9.999065, 0.000048, 0.02},
		{1.057979, 4.499728, 10.000903, 0.000043, 0.02},
		{1.151956, 4.699704, 10.000610, 0.000037, 0.02},
		{1.249968, 4.899709, 9.998443, 0.000032, 0.02},
		{1.351968, 5.099739, 10.001010, 0.000027, 0.02},
		{1.457946, 5.299738, 10.001553, 0.000022, 0.02},
		{1.567930, 5.499723, 10.000188, 0.000017, 0.02},
		{1.681925, 5.699726, 10.000086, 0.000012, 0.02},
		{1.799946, 5.899761, 9.999581, 0.000008, 0.02},
		{1.921945, 6.099799, 10.001657, 0.000003, 0.02},
		{2.047919, 6.299798, 10.001653, -0.000001, 0.02},
		{2.177919, 6.499797, 9.999683, -0.000004, 0.02},
		{2.311933, 6.699827, 10.000197, -0.000008, 0.02},
		{2.449923, 6.899849, 10.001612, -0.000011, 0.02},
		{2.591901, 7.099845, 10.001091, -0.000014, 0.02},
		{2.737894, 7.299843, 10.000204, -0.000017, 0.02},
		{2.887895, 7.499857, 10.000408, -0.000020, 0.02},
		{3.041907, 7.699883, 10.000390, -0.000022, 0.02},
		{3.199892, 7.899898, 10.001593, -0.000025, 0.02},
		{3.361891, 8.099905, 10.000269, -0.000026, 0.02},
		{3.527485, 8.280257, 9.018219, -0.000028, 0.02},
		{3.690986, 8.174520, -5.286480, -0.000030, 0.02},
		{3.850461, 7.974542, -9.999927, -0.000031, 0.02},
		{4.005958, 7.774565, -9.998445, -0.000031, 0.02},
		{4.157445, 7.574581, -9.999543, -0.000032, 0.02},
		{4.304962, 7.374583, -9.998161, -0.000032, 0.02},
		{4.448465, 7.174572, -9.999741, -0.000032, 0.02},
		{4.587937, 6.974593, -10.000389, -0.000032, 0.02},
		{4.723426, 6.774625, -9.998672, -0.000032, 0.02},
		{4.854930, 6.574632, -9.998762, -0.000031, 0.02},
		{4.982433, 6.374629, -9.999324, -0.000030, 0.02},
		{5.105911, 6.174646, -10.000357, -0.000029, 0.02},
		{5.225402, 5.974674, -9.998876, -0.000028, 0.02},
		{5.340897, 5.774688, -9.999148, -0.000027, 0.02},
		{5.452387, 5.574701, -9.999663, -0.000025, 0.02},
		{5.559892, 5.374707, -9.998800, -0.000023, 0.02},
		{5.663396, 5.174697, -9.999444, -0.000022, 0.02},
		{5.762861, 4.974725, -10.001578, -0.000020, 0.02},
		{5.858373, 4.774743, -9.997256, -0.000017, 0.02},
		{5.949871, 4.574726, -10.000486, -0.000015, 0.02},
		{6.037361, 4.374731, -10.000370, -0.000013, 0.02},
		{6.120849, 4.174744, -10.000081, -0.000011, 0.02},
		{6.200344, 3.974748, -9.999821, -0.000009, 0.02},
		{6.275844, 3.774736, -9.999966, -0.000007, 0.02},
		{6.347334, 3.574727, -10.001124, -0.000005, 0.02},
		{6.414828, 3.374722, -10.000267, -0.000004, 0.02},
		{6.478332, 3.174692, -9.999797, -0.000003, 0.02},
		{6.537811, 2.974685, -10.002841, -0.000002, 0.02},
		{6.593329, 2.774650, -9.997248, -0.000001, 0.02},
		{6.644818, 2.574596, -10.003311, -0.000000, 0.02},
		{6.692299, 2.374612, -10.001525, -0.000000, 0.02},
		{6.739590, 2.364876, -0.486850, 0.003144, 0.02},
		{6.860129, 6.026626, 183.077172, 0.067713, 0.02},
		{6.941143, 4.050867, -98.791506, 0.096443, 0.02},
		{7.017280, 3.806482, -12.218138, 0.117879, 0.02},
		{7.094357, 3.853887, 2.370311, 0.136849, 0.02},
		{7.174244, 3.994397, 7.025567, 0.154909, 0.02},
		{7.257824, 4.178601, 9.209335, 0.172802, 0.02},
		{7.345581, 4.388696, 10.506794, 0.190971, 0.02},
		{7.437922, 4.617057, 11.418009, 0.209755, 0.02},
		{7.535148, 4.860252, 12.157120, 0.229438, 0.02},
		{7.637465, 5.116739, 12.826538, 0.250275, 0.02},
		{7.745198, 5.386185, 13.471142, 0.272558, 0.02},
		{7.858568, 5.669028, 14.143532, 0.296580, 0.02},
		{7.977906, 5.966226, 14.858137, 0.322701, 0.02},
		{8.103472, 6.279188, 15.650435, 0.351315, 0.02},
		{8.235694, 6.609769, 16.525639, 0.382936, 0.02},
		{8.374895, 6.960169, 17.520261, 0.418150, 0.02},
		{8.521526, 7.332778, 18.633585, 0.457711, 0.02},
		{8.674599, 7.652596, 15.988787, 0.502122, 0.02},
		{8.826757, 7.608589, -2.200570, 0.549893, 0.02},
		{8.977174, 7.521060, -4.376558, 0.601197, 0.02},
		{9.126013, 7.441929, -3.956509, 0.656547, 0.02},
		{9.273447, 7.371775, -3.507742, 0.716535, 0.02},
		{9.419657, 7.310757, -3.051005, 0.781866, 0.02},
		{9.564856, 7.258708, -2.602029, 0.853395, 0.02},
		{9.709176, 7.215260, -2.172141, 0.932117, 0.02},
		{9.852740, 7.179912, -1.767827, 1.019241, 0.02},
		{9.995818, 7.152074, -1.391552, 1.116363, 0.02},
		{10.138437, 7.131185, -1.044473, 1.225304, 0.02},
		{10.280755, 7.116775, -0.720603, 1.348470, 0.02},
		{10.422933, 7.108523, -0.412598, 1.488949, 0.02},
		{10.565040, 7.106357, -0.108284, 1.650654, 0.02},
		{10.707272, 7.110527, 0.208435, 1.838893, 0.02},
		{10.849683, 7.121697, 0.558614, 2.060569, 0.02},
		{10.992538, 7.141061, 0.967966, 2.325479, 0.02},
		{11.135947, 7.170499, 1.471930, 2.646997, 0.02},
		{11.280172, 7.212793, 2.115144, 3.044643, 0.02},
		{11.425609, 7.272061, 2.963447, 3.547743, 0.02},
		{11.572715, 7.353419, 4.066875, 4.201575, 0.02},
		{11.721331, 7.431279, 3.893280, 5.074119, 0.02},
		{11.870699, 7.468821, 1.877173, 6.268419, 0.02},
		{12.019976, 7.463352, -0.273397, 7.945336, 0.02},
		{12.168251, 7.413549, -2.490114, 10.347917, 0.02},
		{12.314565, 7.316128, -4.871319, 13.788969, 0.02},
		{12.457832, 7.164432, -7.585913, 18.409396, 0.02},
		{12.596750, 6.946136, -10.915239, 23.217274, 0.02},
		{12.725471, 6.435777, -25.516752, 25.000000, 0.02},
		{12.846486, 6.050295, -19.272666, 25.000000, 0.02},
		{12.963491, 5.850289, -10.000347, 25.000000, 0.02},
		{13.076489, 5.650299, -10.000247, 25.000000, 0.02},
		{13.185497, 5.450307, -9.999399, 25.000000, 0.02},
		{13.290498, 5.250313, -10.000222, 25.000000, 0.02},
		{13.391509, 5.050318, -9.999288, 25.000000, 0.02},
		{13.488516, 4.850318, -9.999987, 25.000000, 0.02},
		{13.581520, 4.650327, -9.999806, 25.000000, 0.02},
		{13.670524, 4.450340, -9.999650, 25.000001, 0.02},
		{13.755530, 4.250343, -9.999955, 25.000000, 0.02},
		{13.836542, 4.050329, -10.000017, 25.000000, 0.02},
		{13.913543, 3.850322, -10.000980, 25.000000, 0.02},
		{13.986557, 3.650314, -9.999463, 25.000000, 0.02},
		{14.055563, 3.450301, -10.000691, 25.000000, 0.02},
		{14.120563, 3.250305, -10.000604, 25.000000, 0.02},
		{14.181579, 3.050294, -9.998934, 25.000000, 0.02},
		{14.238590, 2.850265, -10.000490, 25.000000, 0.02},
		{14.291591, 2.650261, -10.001007, 25.000000, 0.02},
		{14.340593, 2.450273, -10.000009, 25.000000, 0.02},
		{14.385594, 2.250287, -10.000312, 25.000000, 0.02},
		{14.426608, 2.050277, -9.998572, 25.000000, 0.02},
		{14.463607, 1.850271, -10.001838, 25.000000, 0.02},
		{14.496614, 1.650283, -9.999112, 25.000000, 0.02},
		{14.525635, 1.450224, -9.997445, 25.000000, 0.02},
		{14.550641, 1.250157, -10.002241, 25.000000, 0.02},
		{14.571646, 1.050130, -9.999779, 25.000000, 0.02},
		{14.588642, 0.850155, -10.002914, 25.000000, 0.02},
		{14.601659, 0.650084, -9.992257, 25.000000, 0.02},
		{14.610665, 0.449875, -10.000907, 25.000000, 0.02},
		{14.615656, 0.249920, -10.011774, 25.000000, 0.02}};
}