package org.usfirst.frc.team4028.robot.util;

public class TwoGearShort {
	// This class contains arrays which hold motion profiles for the drivetrain
	public static final int kNumPoints = 66;		
	// Position (rotations)	Velocity (R/S)	Acceleration (R/S^2) Heading () Duration (ms)
	
	public static double [][]LeftPoints = new double [][]{
		{0.000000, 0.000000, 0.000000, 0.000000, 0.02},
		{0.000902, 0.045108, 2.255201, 0.000100, 0.02},
		{0.003604, 0.135072, 4.497655, 0.000099, 0.02},
		{0.008108, 0.225244, 4.509070, 0.000095, 0.02},
		{0.014420, 0.315511, 4.512532, 0.000087, 0.02},
		{0.022534, 0.405731, 4.511436, 0.000075, 0.02},
		{0.032447, 0.495777, 4.503155, 0.000063, 0.02},
		{0.044162, 0.585704, 4.496048, 0.000051, 0.02},
		{0.057675, 0.675595, 4.494133, 0.000041, 0.02},
		{0.072982, 0.765478, 4.494905, 0.000034, 0.02},
		{0.090095, 0.855388, 4.494324, 0.000027, 0.02},
		{0.109002, 0.945327, 4.496833, 0.000022, 0.02},
		{0.129703, 1.035264, 4.497584, 0.000018, 0.02},
		{0.152208, 1.125214, 4.497367, 0.000015, 0.02},
		{0.176516, 1.215188, 4.498020, 0.000012, 0.02},
		{0.202615, 1.305162, 4.499346, 0.000010, 0.02},
		{0.230521, 1.395139, 4.498426, 0.000008, 0.02},
		{0.260219, 1.485119, 4.499688, 0.000006, 0.02},
		{0.291722, 1.575101, 4.498885, 0.000005, 0.02},
		{0.325019, 1.665086, 4.499815, 0.000003, 0.02},
		{0.360126, 1.755078, 4.498905, 0.000002, 0.02},
		{0.397028, 1.845077, 4.499963, 0.000001, 0.02},
		{0.435734, 1.935075, 4.499384, 0.000000, 0.02},
		{0.476232, 2.025071, 4.500211, -0.000001, 0.02},
		{0.518532, 2.115061, 4.499582, -0.000001, 0.02},
		{0.562633, 2.205056, 4.499794, -0.000002, 0.02},
		{0.608540, 2.295057, 4.499476, -0.000003, 0.02},
		{0.656241, 2.385060, 4.500156, -0.000003, 0.02},
		{0.705728, 2.475045, 4.500487, -0.000004, 0.02},
		{0.757028, 2.565029, 4.499245, -0.000004, 0.02},
		{0.810141, 2.655037, 4.499390, -0.000004, 0.02},
		{0.865044, 2.745046, 4.500319, -0.000005, 0.02},
		{0.921738, 2.835041, 4.500259, -0.000005, 0.02},
		{0.979916, 2.908636, 3.679409, -0.000006, 0.02},
		{1.036873, 2.848201, -3.022178, -0.000006, 0.02},
		{1.092036, 2.758206, -4.499818, -0.000006, 0.02},
		{1.145403, 2.668202, -4.499910, -0.000006, 0.02},
		{1.196966, 2.578199, -4.500275, -0.000007, 0.02},
		{1.246734, 2.488196, -4.499785, -0.000007, 0.02},
		{1.294688, 2.398200, -4.500699, -0.000007, 0.02},
		{1.340864, 2.308197, -4.498962, -0.000007, 0.02},
		{1.385219, 2.218195, -4.501089, -0.000007, 0.02},
		{1.427787, 2.128200, -4.499325, -0.000007, 0.02},
		{1.468550, 2.038197, -4.500245, -0.000007, 0.02},
		{1.507520, 1.948190, -4.499580, -0.000007, 0.02},
		{1.544684, 1.858183, -4.500357, -0.000007, 0.02},
		{1.580041, 1.768192, -4.500438, -0.000007, 0.02},
		{1.613610, 1.678194, -4.499165, -0.000007, 0.02},
		{1.645374, 1.588186, -4.500344, -0.000007, 0.02},
		{1.675333, 1.498194, -4.500353, -0.000007, 0.02},
		{1.703501, 1.408195, -4.499325, -0.000007, 0.02},
		{1.729864, 1.318190, -4.500305, -0.000007, 0.02},
		{1.754423, 1.228199, -4.500449, -0.000007, 0.02},
		{1.777191, 1.138199, -4.499184, -0.000007, 0.02},
		{1.798153, 1.048196, -4.500717, -0.000007, 0.02},
		{1.817317, 0.958199, -4.499863, -0.000006, 0.02},
		{1.834682, 0.868193, -4.499996, -0.000006, 0.02},
		{1.850246, 0.778182, -4.500291, -0.000005, 0.02},
		{1.864009, 0.688175, -4.500552, -0.000004, 0.02},
		{1.875971, 0.598174, -4.500524, -0.000003, 0.02},
		{1.886135, 0.508176, -4.500047, -0.000002, 0.02},
		{1.894500, 0.418171, -4.499068, -0.000001, 0.02},
		{1.901063, 0.328183, -4.499740, -0.000001, 0.02},
		{1.905828, 0.238206, -4.498277, -0.000000, 0.02},
		{1.908792, 0.148228, -4.499875, -0.000000, 0.02},
		{1.909958, 0.057986, -4.486323, -0.000000, 0.02}};
		
	public static double [][]RightPoints = new double[][]{
		{0.000000, 0.000000, 0.000000, 0.000000, 0.02},
		{0.000902, 0.045100, 2.254797, 0.000100, 0.02},
		{0.003602, 0.134956, 4.492226, 0.000099, 0.02},
		{0.008097, 0.224785, 4.491978, 0.000095, 0.02},
		{0.014388, 0.314525, 4.486136, 0.000087, 0.02},
		{0.022474, 0.404313, 4.489849, 0.000075, 0.02},
		{0.032357, 0.494241, 4.497238, 0.000063, 0.02},
		{0.044044, 0.584303, 4.502849, 0.000051, 0.02},
		{0.057533, 0.674426, 4.505701, 0.000041, 0.02},
		{0.072822, 0.764536, 4.506249, 0.000034, 0.02},
		{0.089919, 0.854635, 4.503773, 0.000027, 0.02},
		{0.108814, 0.944722, 4.504241, 0.000022, 0.02},
		{0.129506, 1.034772, 4.503278, 0.000018, 0.02},
		{0.152003, 1.124810, 4.501741, 0.000015, 0.02},
		{0.176303, 1.214852, 4.501411, 0.000012, 0.02},
		{0.202397, 1.304879, 4.502001, 0.000010, 0.02},
		{0.230298, 1.394898, 4.500530, 0.000008, 0.02},
		{0.259991, 1.484912, 4.501376, 0.000006, 0.02},
		{0.291491, 1.574921, 4.500256, 0.000005, 0.02},
		{0.324786, 1.664929, 4.500942, 0.000003, 0.02},
		{0.359890, 1.754939, 4.499836, 0.000002, 0.02},
		{0.396789, 1.844954, 4.500748, 0.000001, 0.02},
		{0.435493, 1.934966, 4.500041, 0.000000, 0.02},
		{0.475989, 2.024973, 4.500774, -0.000001, 0.02},
		{0.518287, 2.114973, 4.500064, -0.000001, 0.02},
		{0.562386, 2.204975, 4.500211, -0.000002, 0.02},
		{0.608292, 2.294984, 4.499837, -0.000003, 0.02},
		{0.655992, 2.384993, 4.500473, -0.000003, 0.02},
		{0.705477, 2.474983, 4.500765, -0.000004, 0.02},
		{0.756777, 2.564973, 4.499494, -0.000004, 0.02},
		{0.809888, 2.654985, 4.499610, -0.000004, 0.02},
		{0.864790, 2.744998, 4.500518, -0.000005, 0.02},
		{0.921483, 2.834996, 4.500439, -0.000005, 0.02},
		{0.979661, 2.908596, 3.679582, -0.000006, 0.02},
		{1.036616, 2.848165, -3.021938, -0.000006, 0.02},
		{1.091779, 2.758174, -4.499598, -0.000006, 0.02},
		{1.145146, 2.668174, -4.499722, -0.000006, 0.02},
		{1.196708, 2.578175, -4.500110, -0.000007, 0.02},
		{1.246476, 2.488174, -4.499638, -0.000007, 0.02},
		{1.294429, 2.398181, -4.500568, -0.000007, 0.02},
		{1.340605, 2.308181, -4.498842, -0.000007, 0.02},
		{1.384960, 2.218180, -4.500975, -0.000007, 0.02},
		{1.427527, 2.128188, -4.499218, -0.000007, 0.02},
		{1.468290, 2.038187, -4.500137, -0.000007, 0.02},
		{1.507260, 1.948182, -4.499477, -0.000007, 0.02},
		{1.544424, 1.858177, -4.500247, -0.000007, 0.02},
		{1.579781, 1.768188, -4.500327, -0.000007, 0.02},
		{1.613350, 1.678192, -4.499043, -0.000007, 0.02},
		{1.645114, 1.588188, -4.500208, -0.000007, 0.02},
		{1.675073, 1.498198, -4.500202, -0.000007, 0.02},
		{1.703241, 1.408203, -4.499148, -0.000007, 0.02},
		{1.729605, 1.318202, -4.500094, -0.000007, 0.02},
		{1.754164, 1.228216, -4.500195, -0.000007, 0.02},
		{1.776933, 1.138223, -4.498861, -0.000007, 0.02},
		{1.797895, 1.048228, -4.500316, -0.000007, 0.02},
		{1.817059, 0.958242, -4.499345, -0.000006, 0.02},
		{1.834426, 0.868249, -4.499320, -0.000006, 0.02},
		{1.849992, 0.778256, -4.499429, -0.000005, 0.02},
		{1.863756, 0.688269, -4.499495, -0.000004, 0.02},
		{1.875721, 0.598291, -4.499386, -0.000003, 0.02},
		{1.885887, 0.508310, -4.499222, -0.000002, 0.02},
		{1.894255, 0.418301, -4.499279, -0.000001, 0.02},
		{1.900820, 0.328278, -4.501425, -0.000001, 0.02},
		{1.905586, 0.238254, -4.500656, -0.000000, 0.02},
		{1.908550, 0.148242, -4.501591, -0.000000, 0.02},
		{1.909716, 0.057987, -4.486943, -0.000000, 0.02}};
}