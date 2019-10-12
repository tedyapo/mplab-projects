const struct {uint16_t delay; uint16_t ftune;} calibration[1024] = {
{0, 656},
{1, 604},
{3, 670},
{3, 356},
{4, 657},
{5, 601},
{7, 668},
{8, 979},
{8, 610},
{10, 668},
{11, 609},
{12, 971},
{12, 603},
{14, 661},
{15, 608},
{16, 881},
{16, 531},
{18, 590},
{19, 540},
{20, 883},
{20, 533},
{22, 595},
{23, 543},
{24, 834},
{25, 780},
{26, 550},
{27, 507},
{28, 852},
{29, 793},
{30, 564},
{31, 514},
{32, 640},
{34, 714},
{35, 661},
{35, 349},
{36, 671},
{37, 612},
{39, 680},
{40, 1001},
{40, 628},
{42, 699},
{43, 652},
{44, 1033},
{44, 653},
{45, 604},
{47, 664},
{48, 960},
{48, 595},
{50, 664},
{51, 608},
{52, 981},
{52, 612},
{54, 681},
{55, 624},
{56, 938},
{56, 577},
{58, 637},
{59, 593},
{60, 962},
{60, 597},
{62, 655},
{63, 608},
{64, 633},
{66, 701},
{67, 652},
{68, 1037},
{68, 657},
{69, 607},
{71, 667},
{72, 987},
{72, 617},
{74, 689},
{75, 630},
{76, 1014},
{76, 638},
{78, 702},
{79, 655},
{80, 946},
{80, 584},
{82, 660},
{83, 611},
{84, 984},
{84, 614},
{86, 684},
{87, 639},
{88, 954},
{88, 590},
{90, 663},
{91, 623},
{92, 1007},
{92, 633},
{94, 707},
{95, 668},
{96, 807},
{97, 753},
{98, 544},
{99, 490},
{100, 844},
{101, 799},
{102, 574},
{103, 528},
{104, 832},
{105, 784},
{106, 556},
{107, 512},
{108, 873},
{108, 524},
{110, 595},
{111, 550},
{112, 824},
{113, 769},
{114, 548},
{115, 507},
{116, 863},
{117, 810},
{118, 582},
{119, 538},
{120, 846},
{121, 795},
{122, 570},
{123, 522},
{124, 877},
{124, 527},
{126, 603},
{127, 558},
{128, 852},
{129, 801},
{130, 576},
{131, 539},
{132, 895},
{132, 542},
{134, 615},
{135, 569},
{136, 893},
{136, 541},
{138, 603},
{139, 565},
{140, 936},
{140, 576},
{142, 652},
{143, 608},
{144, 902},
{144, 548},
{146, 623},
{147, 575},
{148, 952},
{148, 588},
{150, 665},
{151, 617},
{152, 948},
{152, 585},
{154, 656},
{155, 611},
{156, 987},
{156, 616},
{158, 695},
{159, 644},
{160, 785},
{161, 736},
{162, 521},
{163, 479},
{164, 828},
{165, 784},
{166, 559},
{167, 515},
{168, 815},
{169, 774},
{170, 553},
{171, 508},
{172, 860},
{172, 513},
{174, 587},
{175, 542},
{176, 825},
{177, 773},
{178, 550},
{179, 511},
{180, 875},
{180, 526},
{182, 590},
{183, 548},
{184, 867},
{184, 519},
{186, 587},
{187, 541},
{188, 913},
{188, 557},
{190, 629},
{191, 589},
{192, 631},
{194, 702},
{195, 662},
{195, 350},
{196, 682},
{197, 633},
{199, 708},
{199, 388},
{200, 668},
{201, 626},
{203, 698},
{203, 380},
{204, 712},
{205, 672},
{207, 736},
{207, 411},
{208, 683},
{209, 636},
{211, 713},
{211, 392},
{212, 729},
{213, 686},
{215, 759},
{215, 431},
{216, 725},
{217, 682},
{219, 755},
{219, 427},
{220, 773},
{221, 732},
{223, 802},
{223, 466},
{224, 597},
{226, 673},
{227, 636},
{228, 1033},
{228, 653},
{229, 607},
{231, 686},
{231, 369},
{232, 653},
{233, 611},
{235, 688},
{235, 372},
{236, 707},
{237, 665},
{239, 739},
{239, 414},
{240, 680},
{241, 644},
{243, 713},
{243, 392},
{244, 735},
{245, 701},
{247, 776},
{247, 445},
{248, 740},
{249, 688},
{251, 773},
{251, 442},
{252, 793},
{253, 742},
{254, 525},
{255, 485},
{256, 838},
{257, 793},
{258, 566},
{259, 522},
{260, 897},
{260, 544},
{262, 613},
{263, 567},
{264, 884},
{264, 533},
{266, 612},
{267, 564},
{268, 943},
{268, 581},
{270, 660},
{271, 615},
{272, 914},
{272, 558},
{274, 634},
{275, 588},
{276, 979},
{276, 611},
{278, 684},
{279, 647},
{280, 983},
{280, 613},
{282, 694},
{283, 649},
{284, 1047},
{284, 664},
{285, 627},
{287, 705},
{288, 854},
{289, 809},
{290, 586},
{291, 545},
{292, 917},
{292, 560},
{294, 635},
{295, 598},
{296, 919},
{296, 561},
{298, 642},
{299, 596},
{300, 981},
{300, 612},
{302, 684},
{303, 644},
{304, 941},
{304, 580},
{306, 659},
{307, 617},
{308, 1007},
{308, 633},
{310, 711},
{311, 665},
{312, 1003},
{312, 629},
{314, 709},
{315, 661},
{315, 349},
{316, 681},
{317, 633},
{319, 712},
{320, 759},
{321, 719},
{323, 793},
{323, 459},
{324, 821},
{325, 774},
{326, 556},
{327, 511},
{328, 827},
{329, 776},
{330, 559},
{331, 523},
{332, 892},
{332, 540},
{334, 616},
{335, 570},
{336, 868},
{336, 520},
{338, 596},
{339, 560},
{340, 946},
{340, 583},
{342, 651},
{343, 615},
{344, 951},
{344, 588},
{346, 666},
{347, 620},
{348, 1020},
{348, 643},
{349, 604},
{351, 677},
{352, 816},
{353, 774},
{354, 556},
{355, 512},
{356, 882},
{356, 532},
{358, 601},
{359, 560},
{360, 880},
{360, 530},
{362, 603},
{363, 559},
{364, 936},
{364, 576},
{366, 652},
{367, 606},
{368, 902},
{368, 548},
{370, 626},
{371, 584},
{372, 965},
{372, 599},
{374, 678},
{375, 632},
{376, 967},
{376, 601},
{378, 681},
{379, 648},
{380, 1042},
{380, 660},
{381, 617},
{383, 695},
{383, 377},
{384, 648},
{385, 607},
{387, 682},
{387, 366},
{388, 712},
{389, 669},
{391, 742},
{391, 416},
{392, 712},
{393, 670},
{395, 748},
{395, 421},
{396, 773},
{397, 727},
{399, 809},
{399, 472},
{400, 749},
{401, 704},
{403, 783},
{403, 450},
{404, 803},
{405, 760},
{406, 543},
{407, 495},
{408, 807},
{409, 763},
{410, 541},
{411, 498},
{412, 863},
{412, 516},
{414, 587},
{415, 552},
{416, 685},
{417, 649},
{419, 723},
{419, 401},
{420, 746},
{421, 696},
{423, 776},
{423, 444},
{424, 745},
{425, 705},
{427, 780},
{427, 448},
{428, 808},
{429, 762},
{430, 542},
{431, 503},
{432, 785},
{433, 743},
{434, 523},
{435, 484},
{436, 854},
{437, 800},
{438, 574},
{439, 534},
{440, 855},
{441, 813},
{442, 593},
{443, 541},
{444, 923},
{444, 565},
{446, 638},
{447, 594},
{448, 645},
{450, 718},
{451, 669},
{451, 356},
{452, 696},
{453, 651},
{455, 727},
{455, 404},
{456, 697},
{457, 655},
{459, 731},
{459, 407},
{460, 746},
{461, 707},
{463, 780},
{463, 448},
{464, 714},
{465, 676},
{467, 759},
{467, 430},
{468, 778},
{469, 736},
{470, 513},
{471, 482},
{472, 781},
{473, 735},
{474, 524},
{475, 482},
{476, 846},
{477, 798},
{478, 575},
{479, 530},
{480, 669},
{481, 631},
{483, 715},
{483, 394},
{484, 726},
{485, 687},
{487, 765},
{487, 436},
{488, 733},
{489, 690},
{491, 773},
{491, 442},
{492, 798},
{493, 752},
{494, 533},
{495, 504},
{496, 774},
{497, 731},
{499, 810},
{499, 472},
{500, 837},
{501, 785},
{502, 560},
{503, 523},
{504, 838},
{505, 791},
{506, 567},
{507, 528},
{508, 897},
{508, 544},
{510, 622},
{511, 576},
{513, 766},
{514, 553},
{515, 512},
{516, 890},
{516, 538},
{518, 608},
{519, 561},
{520, 884},
{520, 533},
{522, 604},
{523, 565},
{524, 948},
{524, 586},
{526, 664},
{527, 613},
{528, 926},
{528, 568},
{530, 644},
{531, 612},
{532, 1005},
{532, 631},
{534, 707},
{535, 660},
{536, 1015},
{536, 639},
{538, 720},
{539, 680},
{539, 364},
{540, 707},
{541, 663},
{543, 741},
{544, 893},
{544, 541},
{546, 620},
{547, 580},
{548, 977},
{548, 609},
{550, 675},
{551, 641},
{552, 968},
{552, 602},
{554, 684},
{555, 637},
{556, 1037},
{556, 657},
{557, 619},
{559, 688},
{559, 371},
{560, 631},
{562, 714},
{563, 670},
{563, 357},
{564, 688},
{565, 647},
{567, 723},
{567, 401},
{568, 698},
{569, 652},
{571, 732},
{571, 408},
{572, 757},
{573, 708},
{575, 798},
{576, 847},
{577, 789},
{578, 572},
{579, 537},
{580, 914},
{580, 557},
{582, 633},
{583, 596},
{584, 922},
{584, 565},
{586, 645},
{587, 601},
{588, 998},
{588, 625},
{590, 704},
{591, 662},
{592, 979},
{592, 611},
{594, 692},
{595, 650},
{595, 339},
{596, 669},
{597, 626},
{599, 704},
{599, 385},
{600, 680},
{601, 638},
{603, 714},
{603, 393},
{604, 737},
{605, 698},
{607, 775},
{607, 443},
{608, 576},
{610, 652},
{611, 611},
{612, 998},
{612, 626},
{614, 711},
{615, 659},
{616, 1010},
{616, 635},
{618, 716},
{619, 671},
{619, 357},
{620, 693},
{621, 654},
{623, 734},
{623, 410},
{624, 678},
{625, 632},
{627, 719},
{627, 397},
{628, 742},
{629, 705},
{631, 788},
{631, 455},
{632, 762},
{633, 712},
{635, 802},
{635, 466},
{636, 833},
{637, 788},
{638, 568},
{639, 524},
{640, 830},
{641, 779},
{642, 563},
{643, 529},
{644, 904},
{644, 549},
{646, 626},
{647, 585},
{648, 914},
{648, 557},
{650, 636},
{651, 593},
{652, 983},
{652, 613},
{654, 698},
{655, 651},
{656, 974},
{656, 606},
{658, 682},
{659, 641},
{660, 1042},
{660, 661},
{661, 621},
{663, 698},
{663, 380},
{664, 677},
{665, 636},
{667, 713},
{667, 392},
{668, 736},
{669, 697},
{671, 784},
{671, 451},
{672, 587},
{674, 664},
{675, 616},
{676, 1031},
{676, 651},
{677, 606},
{679, 687},
{679, 371},
{680, 655},
{681, 627},
{683, 703},
{683, 384},
{684, 728},
{685, 690},
{687, 772},
{687, 441},
{688, 721},
{689, 680},
{691, 767},
{691, 437},
{692, 795},
{693, 745},
{694, 535},
{695, 497},
{696, 804},
{697, 766},
{698, 548},
{699, 512},
{700, 885},
{700, 534},
{702, 613},
{703, 566},
{704, 618},
{706, 693},
{707, 654},
{707, 343},
{708, 680},
{709, 645},
{711, 724},
{711, 402},
{712, 696},
{713, 658},
{715, 736},
{715, 411},
{716, 763},
{717, 721},
{719, 808},
{719, 471},
{720, 756},
{721, 710},
{723, 798},
{723, 463},
{724, 828},
{725, 786},
{726, 561},
{727, 528},
{728, 847},
{729, 807},
{730, 579},
{731, 549},
{732, 932},
{732, 572},
{734, 662},
{735, 615},
{736, 763},
{737, 723},
{739, 805},
{739, 468},
{740, 846},
{741, 797},
{742, 574},
{743, 536},
{744, 863},
{744, 516},
{746, 590},
{747, 550},
{748, 941},
{748, 580},
{750, 661},
{751, 616},
{752, 923},
{752, 565},
{754, 644},
{755, 607},
{756, 1014},
{756, 638},
{758, 711},
{759, 670},
{759, 357},
{760, 651},
{761, 612},
{763, 680},
{763, 365},
{764, 718},
{765, 671},
{767, 757},
{767, 429},
{768, 788},
{769, 741},
{770, 525},
{771, 488},
{772, 860},
{772, 514},
{774, 592},
{775, 550},
{776, 880},
{776, 530},
{778, 605},
{779, 567},
{780, 959},
{780, 594},
{782, 679},
{783, 638},
{784, 954},
{784, 590},
{786, 663},
{787, 625},
{788, 1038},
{788, 657},
{789, 619},
{791, 701},
{791, 382},
{792, 674},
{793, 633},
{795, 710},
{795, 389},
{796, 737},
{797, 703},
{799, 780},
{799, 447},
{800, 589},
{802, 657},
{803, 623},
{804, 1021},
{804, 644},
{805, 605},
{807, 684},
{807, 368},
{808, 654},
{809, 618},
{811, 692},
{811, 375},
{812, 720},
{813, 678},
{815, 754},
{815, 426},
{816, 701},
{817, 659},
{819, 742},
{819, 416},
{820, 765},
{821, 722},
{823, 801},
{823, 465},
{824, 774},
{825, 734},
{826, 519},
{827, 479},
{828, 843},
{829, 799},
{830, 580},
{831, 536},
{832, 591},
{834, 656},
{835, 620},
{836, 1014},
{836, 638},
{837, 601},
{839, 685},
{839, 369},
{840, 650},
{841, 609},
{843, 685},
{843, 369},
{844, 707},
{845, 667},
{847, 745},
{847, 419},
{848, 687},
{849, 646},
{851, 721},
{851, 399},
{852, 747},
{853, 700},
{855, 777},
{855, 445},
{856, 747},
{857, 703},
{859, 783},
{859, 450},
{860, 802},
{861, 762},
{862, 537},
{863, 500},
{864, 627},
{866, 706},
{867, 663},
{867, 350},
{868, 682},
{869, 640},
{871, 710},
{871, 389},
{872, 681},
{873, 632},
{875, 717},
{875, 396},
{876, 731},
{877, 691},
{879, 771},
{879, 440},
{880, 705},
{881, 659},
{883, 739},
{883, 414},
{884, 758},
{885, 715},
{887, 785},
{887, 452},
{888, 755},
{889, 709},
{891, 793},
{891, 459},
{892, 815},
{893, 758},
{894, 536},
{895, 498},
{896, 780},
{897, 734},
{898, 516},
{899, 476},
{900, 839},
{901, 786},
{902, 561},
{903, 515},
{904, 822},
{905, 772},
{906, 550},
{907, 513},
{908, 868},
{908, 520},
{910, 597},
{911, 548},
{912, 833},
{913, 787},
{914, 556},
{915, 521},
{916, 884},
{916, 534},
{918, 595},
{919, 560},
{920, 875},
{920, 526},
{922, 593},
{923, 558},
{924, 929},
{924, 570},
{926, 641},
{927, 596},
{928, 731},
{929, 693},
{931, 762},
{931, 433},
{932, 781},
{933, 730},
{935, 810},
{935, 472},
{936, 771},
{937, 722},
{939, 803},
{939, 467},
{940, 818},
{941, 765},
{942, 544},
{943, 502},
{944, 782},
{945, 730},
{947, 808},
{947, 471},
{948, 833},
{949, 787},
{950, 553},
{951, 509},
{952, 818},
{953, 768},
{954, 541},
{955, 500},
{956, 861},
{957, 812},
{958, 580},
{959, 531},
{960, 575},
{962, 649},
{963, 602},
{964, 991},
{964, 620},
{966, 689},
{967, 643},
{968, 971},
{968, 604},
{970, 679},
{971, 630},
{972, 1023},
{972, 645},
{973, 595},
{975, 674},
{976, 977},
{976, 609},
{978, 682},
{979, 640},
{980, 1032},
{980, 653},
{981, 600},
{983, 679},
{983, 364},
{984, 647},
{985, 599},
{987, 669},
{987, 356},
{988, 692},
{989, 645},
{991, 714},
{992, 866},
{992, 519},
{994, 590},
{995, 547},
{996, 910},
{996, 555},
{998, 631},
{999, 583},
{1000, 890},
{1000, 538},
{1002, 619},
{1003, 573},
{1004, 948},
{1004, 586},
{1006, 658},
{1007, 613},
{1008, 908},
{1008, 553},
{1010, 629},
{1011, 581},
{1012, 956},
{1012, 592},
{1014, 659},
{1015, 616},
{1016, 938},
{1016, 577},
{1018, 653},
{1019, 612},
{1020, 989},
{1020, 619},
{1022, 691},
{1023, 653},
};
