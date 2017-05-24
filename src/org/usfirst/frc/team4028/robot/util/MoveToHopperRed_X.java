package org.usfirst.frc.team4028.robot.util;

public class MoveToHopperRed_X {
	// This class contains arrays which hold motion profiles for the drivetrain
	public static final int kNumPoints = 98;		
	// Position (rotations)	Velocity (R/S)	Acceleration (R/S^2) Heading () Duration (ms)
	
	public static double [][]LeftPoints = new double [][]{
		{0.000000, 0.000000, 0.000000, 0.000000, 0.02},
		{0.002005, 0.100218, 5.008235, 0.000100, 0.02},
		{0.008005, 0.300098, 9.998318, 0.000099, 0.02},
		{0.018007, 0.500152, 10.003393, 0.000097, 0.02},
		{0.032016, 0.700350, 10.008436, 0.000092, 0.02},
		{0.050026, 0.900582, 10.012687, 0.000082, 0.02},
		{0.072042, 1.100758, 10.008507, 0.000070, 0.02},
		{0.098059, 1.300844, 10.004079, 0.000057, 0.02},
		{0.128075, 1.500828, 9.999395, 0.000043, 0.02},
		{0.162089, 1.700748, 9.996358, 0.000031, 0.02},
		{0.200102, 1.900649, 9.995025, 0.000021, 0.02},
		{0.242115, 2.100557, 9.995068, 0.000012, 0.02},
		{0.288124, 2.300475, 9.995913, 0.000004, 0.02},
		{0.338132, 2.500402, 9.996261, -0.000002, 0.02},
		{0.392137, 2.700337, 9.997074, -0.000007, 0.02},
		{0.450143, 2.900281, 9.997182, -0.000012, 0.02},
		{0.512148, 3.100235, 9.997696, -0.000015, 0.02},
		{0.578148, 3.300188, 9.998166, -0.000018, 0.02},
		{0.648155, 3.500147, 9.997478, -0.000020, 0.02},
		{0.722154, 3.700105, 9.998206, -0.000022, 0.02},
		{0.800156, 3.900049, 9.997111, -0.000023, 0.02},
		{0.882156, 4.099971, 9.996001, -0.000022, 0.02},
		{0.967136, 4.249083, 7.455771, -0.000019, 0.02},
		{1.049097, 4.098094, -7.549479, -0.000012, 0.02},
		{1.127055, 3.897880, -10.010582, -0.000001, 0.02},
		{1.161844, 1.739378, -107.922650, 0.032730, 0.02},
		{1.194647, 1.640165, -4.960681, 0.068957, 0.02},
		{1.245804, 2.557804, 45.881370, 0.093386, 0.02},
		{1.306277, 3.023683, 23.294012, 0.113473, 0.02},
		{1.373174, 3.344698, 16.050232, 0.131616, 0.02},
		{1.445235, 3.603276, 12.929619, 0.148853, 0.02},
		{1.521805, 3.828474, 11.259864, 0.165750, 0.02},
		{1.602464, 4.032855, 10.218725, 0.182665, 0.02},
		{1.686925, 4.222564, 9.484334, 0.199860, 0.02},
		{1.774947, 4.400819, 8.912240, 0.217546, 0.02},
		{1.866327, 4.569330, 8.426101, 0.235910, 0.02},
		{1.960893, 4.728968, 7.983101, 0.255133, 0.02},
		{2.058505, 4.880102, 7.555914, 0.275409, 0.02},
		{2.158957, 5.022701, 7.130123, 0.296928, 0.02},
		{2.262075, 5.156453, 6.688302, 0.319906, 0.02},
		{2.367700, 5.280881, 6.220936, 0.344595, 0.02},
		{2.475612, 5.395323, 5.721831, 0.371274, 0.02},
		{2.585601, 5.498972, 5.181947, 0.400273, 0.02},
		{2.697246, 5.582174, 4.160080, 0.431927, 0.02},
		{2.806583, 5.466669, -5.775060, 0.465500, 0.02},
		{2.912022, 5.272493, -9.709809, 0.500734, 0.02},
		{3.013616, 5.079762, -9.636716, 0.537821, 0.02},
		{3.111422, 4.890715, -9.453069, 0.576959, 0.02},
		{3.205580, 4.707225, -9.173197, 0.618384, 0.02},
		{3.296203, 4.530897, -8.815994, 0.662337, 0.02},
		{3.383459, 4.363157, -8.387670, 0.709098, 0.02},
		{3.467564, 4.205227, -7.896441, 0.759000, 0.02},
		{3.548715, 4.058224, -7.351328, 0.812398, 0.02},
		{3.627202, 3.923149, -6.751677, 0.869754, 0.02},
		{3.703202, 3.800986, -6.109736, 0.931497, 0.02},
		{3.777061, 3.692647, -5.416492, 0.998256, 0.02},
		{3.849038, 3.598973, -4.683900, 1.070671, 0.02},
		{3.919468, 3.520814, -3.907202, 1.149561, 0.02},
		{3.988652, 3.458997, -3.090684, 1.235836, 0.02},
		{4.056937, 3.414351, -2.232339, 1.330623, 0.02},
		{4.124689, 3.387700, -1.332575, 1.435282, 0.02},
		{4.192289, 3.379900, -0.389988, 1.551465, 0.02},
		{4.260104, 3.391858, 0.598071, 1.681137, 0.02},
		{4.328604, 3.424569, 1.635356, 1.826897, 0.02},
		{4.398187, 3.479168, 2.729924, 1.991792, 0.02},
		{4.469327, 3.556970, 3.890111, 2.179789, 0.02},
		{4.542515, 3.659590, 5.131283, 2.395924, 0.02},
		{4.618290, 3.789061, 6.474008, 2.646768, 0.02},
		{4.697127, 3.941338, 7.612900, 2.940498, 0.02},
		{4.776378, 3.962103, 1.038114, 3.273688, 0.02},
		{4.854775, 3.919562, -2.126858, 3.645542, 0.02},
		{4.932240, 3.873230, -2.316570, 4.059575, 0.02},
		{5.008654, 3.821379, -2.593068, 4.519058, 0.02},
		{5.083904, 3.762772, -2.930533, 5.027003, 0.02},
		{5.157864, 3.696575, -3.308580, 5.585701, 0.02},
		{5.230288, 3.622327, -3.713543, 6.195292, 0.02},
		{5.301078, 3.539807, -4.126408, 6.855328, 0.02},
		{5.370066, 3.448923, -4.543524, 7.562525, 0.02},
		{5.437052, 3.349774, -4.958163, 8.310305, 0.02},
		{5.501927, 3.242505, -5.361402, 9.089805, 0.02},
		{5.564467, 3.127344, -5.758651, 9.887393, 0.02},
		{5.624548, 3.004576, -6.139521, 10.687440, 0.02},
		{5.682057, 2.874408, -6.506014, 11.472272, 0.02},
		{5.736808, 2.737124, -6.863150, 12.222170, 0.02},
		{5.788670, 2.593056, -7.203221, 12.918892, 0.02},
		{5.837526, 2.442479, -7.527920, 13.546894, 0.02},
		{5.883223, 2.285724, -7.840727, 14.094298, 0.02},
		{5.925668, 2.123121, -8.133455, 14.555106, 0.02},
		{5.964780, 1.954898, -8.408068, 14.928764, 0.02},
		{6.000442, 1.781345, -8.669091, 15.219269, 0.02},
		{6.032498, 1.603004, -8.918405, 15.434462, 0.02},
		{6.060899, 1.420348, -9.134656, 15.585710, 0.02},
		{6.085561, 1.233743, -9.335019, 15.685327, 0.02},
		{6.106461, 1.043465, -9.499909, 15.745863, 0.02},
		{6.123435, 0.850303, -9.676374, 15.778583, 0.02},
		{6.136534, 0.654773, -9.774057, 15.793657, 0.02},
		{6.145718, 0.456540, -9.853631, 15.798920, 0.02},
		{6.150859, 0.256935, -9.977219, 15.799955, 0.02}};
	
	public static double [][]RightPoints = new double [][]{
		{0.000000, 0.000000, 0.000000, 0.000000, 0.02},
		{0.002005, 0.100214, 5.007998, 0.000100, 0.02},
		{0.008003, 0.300030, 9.995148, 0.000099, 0.02},
		{0.018000, 0.499875, 9.992928, 0.000097, 0.02},
		{0.031996, 0.699693, 9.989455, 0.000092, 0.02},
		{0.049983, 0.899468, 9.989800, 0.000082, 0.02},
		{0.071970, 1.099277, 9.990190, 0.000070, 0.02},
		{0.097954, 1.299200, 9.995958, 0.000057, 0.02},
		{0.127938, 1.499217, 10.001019, 0.000043, 0.02},
		{0.161922, 1.699285, 10.003792, 0.000031, 0.02},
		{0.199910, 1.899378, 10.004602, 0.000021, 0.02},
		{0.241901, 2.099477, 10.004609, 0.000012, 0.02},
		{0.287892, 2.299566, 10.004474, 0.000004, 0.02},
		{0.337885, 2.499640, 10.003622, -0.000002, 0.02},
		{0.391878, 2.699700, 10.003305, -0.000007, 0.02},
		{0.449873, 2.899750, 10.002492, -0.000012, 0.02},
		{0.511869, 3.099797, 10.002318, -0.000015, 0.02},
		{0.577862, 3.299834, 10.002354, -0.000018, 0.02},
		{0.647863, 3.499874, 10.001546, -0.000020, 0.02},
		{0.721859, 3.699919, 10.002561, -0.000022, 0.02},
		{0.799859, 3.899971, 10.002500, -0.000023, 0.02},
		{0.881861, 4.100053, 10.003996, -0.000022, 0.02},
		{0.966848, 4.249450, 7.470057, -0.000019, 0.02},
		{1.048827, 4.098967, -7.524199, -0.000012, 0.02},
		{1.126811, 3.899185, -9.988963, -0.000001, 0.02},
		{1.240774, 5.698032, 89.940282, 0.032730, 0.02},
		{1.361006, 6.011587, 15.677743, 0.068957, 0.02},
		{1.470891, 5.494157, -25.871131, 0.093386, 0.02},
		{1.579457, 5.428338, -3.290977, 0.113473, 0.02},
		{1.689608, 5.507349, 3.950424, 0.131616, 0.02},
		{1.802577, 5.648775, 7.071691, 0.148853, 0.02},
		{1.919049, 5.823566, 8.739541, 0.165750, 0.02},
		{2.039436, 6.019189, 9.780821, 0.182665, 0.02},
		{2.164041, 6.229503, 10.514495, 0.199860, 0.02},
		{2.293073, 6.451272, 11.087834, 0.217546, 0.02},
		{2.426720, 6.682747, 11.574467, 0.235910, 0.02},
		{2.565160, 6.923049, 12.016879, 0.255133, 0.02},
		{2.708612, 7.171886, 12.440590, 0.275409, 0.02},
		{2.857195, 7.429277, 12.869851, 0.296928, 0.02},
		{3.011088, 7.695469, 13.311011, 0.319906, 0.02},
		{3.170520, 7.970998, 13.775380, 0.344595, 0.02},
		{3.335659, 8.256535, 14.276180, 0.371274, 0.02},
		{3.506731, 8.552860, 14.814908, 0.400273, 0.02},
		{3.683670, 8.846840, 14.698825, 0.431927, 0.02},
		{3.860632, 8.847802, 0.048117, 0.465500, 0.02},
		{4.035197, 8.729165, -5.932481, 0.500734, 0.02},
		{4.207477, 8.614121, -5.752307, 0.537821, 0.02},
		{4.377544, 8.504004, -5.506273, 0.576959, 0.02},
		{4.545564, 8.399811, -5.208884, 0.618384, 0.02},
		{4.711618, 8.302271, -4.876774, 0.662337, 0.02},
		{4.875842, 8.211919, -4.517993, 0.709098, 0.02},
		{5.038425, 8.129094, -4.141181, 0.759000, 0.02},
		{5.199480, 8.054017, -3.754494, 0.812398, 0.02},
		{5.359265, 7.986792, -3.360213, 0.869754, 0.02},
		{5.517773, 7.927484, -2.966177, 0.931497, 0.02},
		{5.675308, 7.876098, -2.569068, 0.998256, 0.02},
		{5.831954, 7.832604, -2.174780, 1.070671, 0.02},
		{5.987923, 7.796990, -1.780358, 1.149561, 0.02},
		{6.143318, 7.769264, -1.386207, 1.235836, 0.02},
		{6.298304, 7.749487, -0.988892, 1.330623, 0.02},
		{6.453056, 7.737783, -0.585233, 1.435282, 0.02},
		{6.607746, 7.734377, -0.170306, 1.551465, 0.02},
		{6.762490, 7.739621, 0.262288, 1.681137, 0.02},
		{6.917589, 7.754029, 0.720317, 1.826897, 0.02},
		{7.073155, 7.778320, 1.214563, 1.991792, 0.02},
		{7.229426, 7.813461, 1.757031, 2.179789, 0.02},
		{7.386631, 7.860746, 2.364414, 2.395924, 0.02},
		{7.545057, 7.921895, 3.057667, 2.646768, 0.02},
		{7.704794, 7.985814, 3.195549, 2.940498, 0.02},
		{7.859896, 7.754291, -11.574939, 3.273688, 0.02},
		{8.007845, 7.396884, -17.868984, 3.645542, 0.02},
		{8.148712, 7.043274, -17.680305, 4.059575, 0.02},
		{8.282592, 6.695224, -17.405684, 4.519058, 0.02},
		{8.409661, 6.353935, -17.065779, 5.027003, 0.02},
		{8.530110, 6.020112, -16.684651, 5.585701, 0.02},
		{8.643962, 5.694379, -16.291760, 6.195292, 0.02},
		{8.751493, 5.377007, -15.870006, 6.855328, 0.02},
		{8.852866, 5.067900, -15.453093, 7.562525, 0.02},
		{8.948193, 4.767064, -15.043933, 8.310305, 0.02},
		{9.037713, 4.474298, -14.632790, 9.089805, 0.02},
		{9.121493, 4.189412, -14.245690, 9.887393, 0.02},
		{9.199723, 3.912243, -13.861024, 10.687440, 0.02},
		{9.272598, 3.642379, -13.488243, 11.472272, 0.02},
		{9.340200, 3.379562, -13.138819, 12.222170, 0.02},
		{9.402673, 3.123596, -12.798042, 12.918892, 0.02},
		{9.460163, 2.874145, -12.470938, 13.546894, 0.02},
		{9.512762, 2.630951, -12.164331, 14.094298, 0.02},
		{9.560617, 2.393712, -11.866840, 14.555106, 0.02},
		{9.603871, 2.161944, -11.584153, 14.928764, 0.02},
		{9.642614, 1.935226, -11.324691, 15.219269, 0.02},
		{9.676877, 1.713399, -11.093031, 15.434462, 0.02},
		{9.706793, 1.496126, -10.865849, 15.585710, 0.02},
		{9.732438, 1.282873, -10.668114, 15.685327, 0.02},
		{9.753929, 1.072960, -10.480227, 15.745863, 0.02},
		{9.771220, 0.866204, -10.357320, 15.778583, 0.02},
		{9.784464, 0.662062, -10.204608, 15.793657, 0.02},
		{9.793700, 0.459068, -10.090294, 15.798920, 0.02},
		{9.798850, 0.257434, -10.078569, 15.799955, 0.02}};
}