package org.usfirst.frc.team4028.robot.util;

public class TwoGearShort {
	// This class contains arrays which hold motion profiles for the drivetrain
	public static final int kNumPoints = 75;		
	// Position (rotations)	Velocity (R/S)	Acceleration (R/S^2) Heading () Duration (ms)
	
	public static double [][]LeftPoints = new double [][]{
		{0.000000, 0.000000, 0.000000, 0.000000, 0.02},
		{0.001207, 0.060289, 3.012530, 0.000100, 0.02},
		{0.004814, 0.180356, 6.002523, 0.000095, 0.02},
		{0.010818, 0.300324, 6.000894, 0.000091, 0.02},
		{0.019163, 0.417248, 5.846136, 0.000137, 0.02},
		{0.029673, 0.525650, 5.421783, 0.000375, 0.02},
		{0.042063, 0.619394, 4.686198, 0.001046, 0.02},
		{0.055981, 0.695909, 3.826039, 0.002437, 0.02},
		{0.071158, 0.758828, 3.145811, 0.004771, 0.02},
		{0.087475, 0.816008, 2.859449, 0.008142, 0.02},
		{0.104974, 0.874958, 2.947557, 0.012524, 0.02},
		{0.123778, 0.940014, 3.252165, 0.017815, 0.02},
		{0.144021, 1.012269, 3.613229, 0.023896, 0.02},
		{0.165839, 1.090885, 3.930617, 0.030664, 0.02},
		{0.189321, 1.174264, 4.169645, 0.038039, 0.02},
		{0.214540, 1.260732, 4.322710, 0.045976, 0.02},
		{0.241517, 1.348806, 4.403564, 0.054447, 0.02},
		{0.270261, 1.437235, 4.421503, 0.063450, 0.02},
		{0.300761, 1.525011, 4.388802, 0.072999, 0.02},
		{0.332982, 1.611276, 4.313815, 0.083120, 0.02},
		{0.366891, 1.695286, 4.200197, 0.093861, 0.02},
		{0.402417, 1.776341, 4.052841, 0.105273, 0.02},
		{0.439496, 1.853745, 3.869667, 0.117428, 0.02},
		{0.478029, 1.926768, 3.651492, 0.130404, 0.02},
		{0.517916, 1.994620, 3.392997, 0.144303, 0.02},
		{0.559055, 2.056437, 3.090098, 0.159248, 0.02},
		{0.601271, 2.111197, 2.738487, 0.175367, 0.02},
		{0.644422, 2.157718, 2.326255, 0.192835, 0.02},
		{0.688323, 2.194639, 1.845672, 0.211857, 0.02},
		{0.732726, 2.220319, 1.284143, 0.232665, 0.02},
		{0.777385, 2.232831, 0.625564, 0.255560, 0.02},
		{0.821978, 2.229899, -0.146611, 0.280896, 0.02},
		{0.866157, 2.208845, -1.052645, 0.309128, 0.02},
		{0.909468, 2.165338, -2.175131, 0.340790, 0.02},
		{0.949913, 2.022402, -7.147443, 0.375186, 0.02},
		{0.986445, 1.826751, -9.783362, 0.411841, 0.02},
		{1.019067, 1.631067, -9.783932, 0.450976, 0.02},
		{1.047818, 1.437330, -9.685424, 0.492814, 0.02},
		{1.072795, 1.248900, -9.421697, 0.537606, 0.02},
		{1.094180, 1.069198, -8.984947, 0.585713, 0.02},
		{1.112172, 0.899732, -8.474529, 0.637533, 0.02},
		{1.127009, 0.741878, -7.892688, 0.693556, 0.02},
		{1.138950, 0.597059, -7.240978, 0.754325, 0.02},
		{1.148286, 0.466784, -6.514107, 0.820485, 0.02},
		{1.155339, 0.352602, -5.707863, 0.892822, 0.02},
		{1.160462, 0.256171, -4.822246, 0.972192, 0.02},
		{1.164046, 0.179214, -3.847419, 1.059721, 0.02},
		{1.166517, 0.123545, -2.783743, 1.156652, 0.02},
		{1.168340, 0.091119, -1.621271, 1.264564, 0.02},
		{1.170021, 0.084075, -0.352145, 1.385327, 0.02},
		{1.172118, 0.104855, 1.039278, 1.521096, 0.02},
		{1.175244, 0.156311, 2.572614, 1.674607, 0.02},
		{1.179874, 0.231491, 3.759243, 1.842259, 0.02},
		{1.186276, 0.320113, 4.431291, 2.019706, 0.02},
		{1.194624, 0.417312, 4.858685, 2.205155, 0.02},
		{1.204952, 0.516330, 4.950371, 2.396012, 0.02},
		{1.217181, 0.611554, 4.761932, 2.589113, 0.02},
		{1.231156, 0.698657, 4.354689, 2.780892, 0.02},
		{1.246643, 0.774439, 3.789593, 2.967266, 0.02},
		{1.263375, 0.836674, 3.111993, 3.144248, 0.02},
		{1.281058, 0.883947, 2.363035, 3.308122, 0.02},
		{1.299365, 0.915430, 1.574295, 3.455602, 0.02},
		{1.317979, 0.930766, 0.766879, 3.584470, 0.02},
		{1.336575, 0.929962, -0.040197, 3.693487, 0.02},
		{1.354850, 0.913283, -0.833517, 3.782601, 0.02},
		{1.372474, 0.881225, -1.603010, 3.852601, 0.02},
		{1.389156, 0.834506, -2.336971, 3.905278, 0.02},
		{1.404646, 0.773967, -3.024891, 3.943043, 0.02},
		{1.418652, 0.700686, -3.666121, 3.968484, 0.02},
		{1.430972, 0.615909, -4.238285, 3.984411, 0.02},
		{1.441402, 0.520928, -4.743737, 3.993426, 0.02},
		{1.449752, 0.417355, -5.176774, 3.997830, 0.02},
		{1.455883, 0.307048, -5.524155, 3.999535, 0.02},
		{1.459731, 0.191405, -5.753402, 3.999964, 0.02},
		{1.461194, 0.066196, -5.664526, 4.000000, 0.02}};
		
	public static double [][]RightPoints = new double[][]{
		{0.000000, 0.000000, 0.000000, 0.000000, 0.02},
		{0.001206, 0.060237, 3.009941, 0.000100, 0.02},
		{0.004802, 0.179814, 5.977988, 0.000095, 0.02},
		{0.010796, 0.299813, 6.002455, 0.000091, 0.02},
		{0.019253, 0.422840, 6.151319, 0.000137, 0.02},
		{0.030337, 0.554402, 6.580138, 0.000375, 0.02},
		{0.044353, 0.700646, 7.310600, 0.001046, 0.02},
		{0.061635, 0.864148, 8.175723, 0.002437, 0.02},
		{0.082460, 1.041225, 8.853532, 0.004771, 0.02},
		{0.106937, 1.224030, 9.141603, 0.008142, 0.02},
		{0.135038, 1.405059, 9.051617, 0.012524, 0.02},
		{0.166644, 1.580023, 8.746582, 0.017815, 0.02},
		{0.201595, 1.747773, 8.388574, 0.023896, 0.02},
		{0.239780, 1.909145, 8.068274, 0.030664, 0.02},
		{0.281088, 2.065750, 7.831529, 0.038039, 0.02},
		{0.325481, 2.219280, 7.675242, 0.045976, 0.02},
		{0.372907, 2.371227, 7.597129, 0.054447, 0.02},
		{0.423362, 2.522798, 7.578617, 0.063450, 0.02},
		{0.476863, 2.675018, 7.611024, 0.072999, 0.02},
		{0.533430, 2.828735, 7.686837, 0.083120, 0.02},
		{0.593128, 2.984714, 7.798396, 0.093861, 0.02},
		{0.656000, 3.143661, 7.947516, 0.105273, 0.02},
		{0.722134, 3.306266, 8.129183, 0.117428, 0.02},
		{0.791593, 3.473242, 8.349510, 0.130404, 0.02},
		{0.864491, 3.645359, 8.606889, 0.144303, 0.02},
		{0.940981, 3.823547, 8.907274, 0.159248, 0.02},
		{1.021142, 4.008785, 9.263558, 0.175367, 0.02},
		{1.105181, 4.202219, 9.672417, 0.192835, 0.02},
		{1.193303, 4.405297, 10.151994, 0.211857, 0.02},
		{1.285687, 4.619607, 10.716487, 0.232665, 0.02},
		{1.382633, 4.847065, 11.372302, 0.255560, 0.02},
		{1.484422, 5.089959, 12.145894, 0.280896, 0.02},
		{1.591446, 5.350964, 13.049653, 0.309128, 0.02},
		{1.704057, 5.629929, 13.946764, 0.340790, 0.02},
		{1.818278, 5.711579, 4.082898, 0.375186, 0.02},
		{1.931613, 5.667236, -2.217362, 0.411841, 0.02},
		{2.044075, 5.622912, -2.216110, 0.450976, 0.02},
		{2.155626, 5.576746, -2.307966, 0.492814, 0.02},
		{2.266257, 5.531682, -2.253268, 0.537606, 0.02},
		{2.376137, 5.493876, -1.890238, 0.585713, 0.02},
		{2.485389, 5.463394, -1.524353, 0.637533, 0.02},
		{2.594181, 5.439591, -1.190163, 0.693556, 0.02},
		{2.702614, 5.421698, -0.894626, 0.754325, 0.02},
		{2.810786, 5.408862, -0.641826, 0.820485, 0.02},
		{2.918812, 5.400167, -0.434656, 0.892822, 0.02},
		{3.026691, 5.394693, -0.273739, 0.972192, 0.02},
		{3.134534, 5.391556, -0.156824, 1.059721, 0.02},
		{3.242322, 5.389971, -0.079259, 1.156652, 0.02},
		{3.350110, 5.389312, -0.032956, 1.264564, 0.02},
		{3.457916, 5.389197, -0.005757, 1.385327, 0.02},
		{3.565679, 5.389576, 0.018953, 1.521096, 0.02},
		{3.673460, 5.388581, -0.049773, 1.674607, 0.02},
		{3.777344, 5.194607, -9.699427, 1.842259, 0.02},
		{3.874661, 4.866074, -16.427333, 2.019706, 0.02},
		{3.965263, 4.528915, -16.853503, 2.205155, 0.02},
		{4.049070, 4.189910, -16.948485, 2.396012, 0.02},
		{4.126154, 3.854737, -16.761102, 2.589113, 0.02},
		{4.196715, 3.527676, -16.351313, 2.780892, 0.02},
		{4.260945, 3.211925, -15.789584, 2.967266, 0.02},
		{4.319135, 2.909739, -15.110695, 3.144248, 0.02},
		{4.371598, 2.622459, -14.360107, 3.308122, 0.02},
		{4.418613, 2.350965, -13.575950, 3.455602, 0.02},
		{4.460522, 2.095659, -12.766513, 3.584470, 0.02},
		{4.497646, 1.856499, -11.960064, 3.693487, 0.02},
		{4.530325, 1.633140, -11.162160, 3.782601, 0.02},
		{4.558827, 1.425143, -10.400312, 3.852601, 0.02},
		{4.583455, 1.231921, -9.665367, 3.905278, 0.02},
		{4.604518, 1.052431, -8.968354, 3.943043, 0.02},
		{4.622221, 0.885699, -8.341404, 3.968484, 0.02},
		{4.636834, 0.730528, -7.757455, 3.984411, 0.02},
		{4.648554, 0.585359, -7.250400, 3.993426, 0.02},
		{4.657533, 0.448754, -6.827734, 3.997830, 0.02},
		{4.663907, 0.319209, -6.487607, 3.999535, 0.02},
		{4.667815, 0.194443, -6.207244, 3.999964, 0.02},
		{4.669283, 0.066428, -5.791489, 4.000000, 0.02}};
}