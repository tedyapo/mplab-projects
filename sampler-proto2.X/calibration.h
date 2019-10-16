const struct {uint16_t delay; uint16_t ftune;} calibration[1024] = {
{0, 632},
{2, 716},
{3, 670},
{3, 357},
{4, 661},
{5, 614},
{7, 684},
{8, 1003},
{8, 631},
{10, 699},
{11, 648},
{12, 1007},
{12, 634},
{14, 705},
{15, 650},
{16, 923},
{16, 566},
{18, 633},
{19, 579},
{20, 917},
{20, 561},
{22, 621},
{23, 574},
{24, 859},
{25, 797},
{26, 580},
{27, 524},
{28, 849},
{29, 786},
{30, 568},
{31, 514},
{32, 623},
{34, 683},
{35, 633},
{36, 1000},
{36, 628},
{38, 689},
{39, 643},
{40, 937},
{40, 578},
{42, 654},
{43, 603},
{44, 953},
{44, 591},
{46, 667},
{47, 609},
{48, 893},
{48, 542},
{50, 607},
{51, 567},
{52, 916},
{52, 560},
{54, 628},
{55, 587},
{56, 889},
{56, 538},
{58, 612},
{59, 562},
{60, 911},
{60, 557},
{62, 632},
{63, 582},
{64, 614},
{66, 684},
{67, 637},
{68, 1001},
{68, 629},
{70, 693},
{71, 648},
{72, 960},
{72, 596},
{74, 666},
{75, 616},
{76, 967},
{76, 602},
{78, 673},
{79, 630},
{80, 895},
{80, 543},
{82, 614},
{83, 564},
{84, 918},
{84, 562},
{86, 634},
{87, 582},
{88, 869},
{88, 522},
{90, 601},
{91, 552},
{92, 906},
{92, 552},
{94, 617},
{95, 571},
{96, 689},
{97, 650},
{99, 731},
{99, 408},
{100, 725},
{101, 676},
{103, 758},
{103, 430},
{104, 714},
{105, 671},
{107, 755},
{107, 428},
{108, 752},
{109, 710},
{111, 797},
{111, 462},
{112, 710},
{113, 674},
{115, 754},
{115, 427},
{116, 755},
{117, 706},
{119, 787},
{119, 454},
{120, 754},
{121, 699},
{123, 781},
{123, 450},
{124, 788},
{125, 746},
{126, 520},
{127, 479},
{128, 765},
{129, 717},
{131, 793},
{131, 459},
{132, 799},
{133, 750},
{134, 538},
{135, 488},
{136, 782},
{137, 733},
{138, 522},
{139, 477},
{140, 829},
{141, 764},
{142, 556},
{143, 511},
{144, 784},
{145, 733},
{146, 521},
{147, 482},
{148, 825},
{149, 780},
{150, 562},
{151, 519},
{152, 826},
{153, 779},
{154, 568},
{155, 519},
{156, 878},
{156, 529},
{158, 599},
{159, 564},
{160, 687},
{161, 647},
{163, 728},
{163, 405},
{164, 735},
{165, 687},
{167, 765},
{167, 436},
{168, 724},
{169, 682},
{171, 760},
{171, 432},
{172, 765},
{173, 715},
{175, 800},
{175, 465},
{176, 717},
{177, 675},
{179, 753},
{179, 426},
{180, 756},
{181, 715},
{183, 812},
{183, 475},
{184, 751},
{185, 711},
{187, 798},
{187, 463},
{188, 799},
{189, 756},
{190, 541},
{191, 495},
{192, 533},
{194, 610},
{195, 566},
{196, 941},
{196, 580},
{198, 651},
{199, 609},
{200, 922},
{200, 565},
{202, 636},
{203, 599},
{204, 976},
{204, 609},
{206, 690},
{207, 639},
{208, 937},
{208, 578},
{210, 654},
{211, 623},
{212, 998},
{212, 627},
{214, 698},
{215, 655},
{216, 997},
{216, 626},
{218, 702},
{219, 652},
{220, 1046},
{220, 665},
{221, 618},
{223, 700},
{224, 835},
{225, 791},
{226, 570},
{227, 528},
{228, 890},
{228, 539},
{230, 614},
{231, 569},
{232, 881},
{232, 532},
{234, 602},
{235, 569},
{236, 937},
{236, 577},
{238, 658},
{239, 614},
{240, 898},
{240, 546},
{242, 630},
{243, 585},
{244, 957},
{244, 594},
{246, 675},
{247, 639},
{248, 966},
{248, 601},
{250, 680},
{251, 647},
{252, 1032},
{252, 654},
{253, 618},
{255, 699},
{255, 382},
{256, 723},
{257, 684},
{259, 763},
{259, 434},
{260, 781},
{261, 737},
{262, 524},
{263, 485},
{264, 788},
{265, 741},
{266, 521},
{267, 491},
{268, 838},
{269, 788},
{270, 571},
{271, 534},
{272, 800},
{273, 757},
{274, 539},
{275, 498},
{276, 851},
{277, 808},
{278, 584},
{279, 541},
{280, 846},
{281, 805},
{282, 584},
{283, 544},
{284, 907},
{284, 553},
{286, 626},
{287, 593},
{288, 723},
{289, 670},
{291, 765},
{291, 436},
{292, 776},
{293, 729},
{294, 518},
{295, 481},
{296, 782},
{297, 730},
{298, 522},
{299, 485},
{300, 840},
{301, 788},
{302, 578},
{303, 536},
{304, 811},
{305, 769},
{306, 557},
{307, 516},
{308, 881},
{308, 532},
{310, 610},
{311, 565},
{312, 886},
{312, 536},
{314, 614},
{315, 573},
{316, 940},
{316, 580},
{318, 656},
{319, 613},
{320, 657},
{321, 615},
{323, 697},
{323, 379},
{324, 704},
{325, 665},
{327, 743},
{327, 418},
{328, 703},
{329, 659},
{331, 738},
{331, 413},
{332, 760},
{333, 706},
{335, 790},
{335, 457},
{336, 722},
{337, 682},
{339, 771},
{339, 441},
{340, 783},
{341, 737},
{342, 523},
{343, 487},
{344, 786},
{345, 740},
{346, 534},
{347, 492},
{348, 854},
{349, 806},
{350, 584},
{351, 542},
{352, 677},
{353, 634},
{355, 724},
{355, 402},
{356, 740},
{357, 696},
{359, 783},
{359, 451},
{360, 747},
{361, 706},
{363, 797},
{363, 463},
{364, 810},
{365, 767},
{366, 547},
{367, 507},
{368, 782},
{369, 736},
{370, 526},
{371, 487},
{372, 842},
{373, 798},
{374, 573},
{375, 530},
{376, 842},
{377, 792},
{378, 575},
{379, 534},
{380, 899},
{380, 547},
{382, 623},
{383, 579},
{384, 887},
{384, 536},
{386, 616},
{387, 567},
{388, 944},
{388, 583},
{390, 669},
{391, 619},
{392, 947},
{392, 586},
{394, 670},
{395, 628},
{396, 1015},
{396, 640},
{397, 600},
{399, 683},
{400, 988},
{400, 618},
{402, 705},
{403, 661},
{403, 349},
{404, 676},
{405, 637},
{407, 718},
{407, 397},
{408, 685},
{409, 640},
{411, 731},
{411, 408},
{412, 742},
{413, 707},
{415, 787},
{415, 454},
{416, 582},
{418, 656},
{419, 624},
{420, 997},
{420, 626},
{422, 713},
{423, 668},
{424, 999},
{424, 628},
{426, 712},
{427, 671},
{427, 358},
{428, 680},
{429, 633},
{431, 716},
{431, 395},
{432, 649},
{433, 616},
{435, 688},
{435, 372},
{436, 705},
{437, 665},
{439, 750},
{439, 423},
{440, 715},
{441, 667},
{443, 756},
{443, 429},
{444, 772},
{445, 726},
{446, 519},
{447, 482},
{448, 519},
{450, 600},
{451, 556},
{452, 930},
{452, 572},
{454, 652},
{455, 616},
{456, 941},
{456, 581},
{458, 664},
{459, 619},
{460, 1008},
{460, 635},
{462, 726},
{463, 674},
{464, 983},
{464, 615},
{466, 695},
{467, 652},
{468, 1045},
{468, 665},
{469, 614},
{471, 704},
{471, 385},
{472, 675},
{473, 635},
{475, 713},
{475, 393},
{476, 720},
{477, 678},
{479, 764},
{479, 436},
{480, 560},
{482, 635},
{483, 598},
{484, 965},
{484, 600},
{486, 685},
{487, 641},
{488, 971},
{488, 605},
{490, 688},
{491, 639},
{492, 1030},
{492, 652},
{493, 616},
{495, 699},
{495, 381},
{496, 630},
{498, 724},
{499, 676},
{499, 362},
{500, 687},
{501, 651},
{503, 730},
{503, 407},
{504, 698},
{505, 659},
{507, 745},
{507, 419},
{508, 758},
{509, 719},
{511, 806},
{512, 709},
{513, 665},
{515, 758},
{515, 430},
{516, 770},
{517, 727},
{519, 814},
{519, 477},
{520, 777},
{521, 731},
{522, 517},
{523, 478},
{524, 828},
{525, 786},
{526, 566},
{527, 526},
{528, 801},
{529, 753},
{530, 541},
{531, 499},
{532, 856},
{533, 812},
{534, 592},
{535, 550},
{536, 863},
{536, 517},
{538, 590},
{539, 559},
{540, 920},
{540, 564},
{542, 642},
{543, 603},
{544, 743},
{545, 700},
{547, 788},
{547, 455},
{548, 800},
{549, 740},
{550, 542},
{551, 503},
{552, 812},
{553, 773},
{554, 554},
{555, 511},
{556, 876},
{556, 527},
{558, 608},
{559, 570},
{560, 858},
{560, 513},
{562, 595},
{563, 558},
{564, 935},
{564, 575},
{566, 656},
{567, 616},
{568, 950},
{568, 588},
{570, 671},
{571, 627},
{572, 1015},
{572, 640},
{573, 598},
{575, 673},
{576, 719},
{577, 685},
{579, 771},
{579, 441},
{580, 773},
{581, 733},
{582, 521},
{583, 483},
{584, 783},
{585, 739},
{586, 523},
{587, 486},
{588, 840},
{589, 799},
{590, 578},
{591, 539},
{592, 808},
{593, 774},
{594, 551},
{595, 516},
{596, 882},
{596, 532},
{598, 611},
{599, 572},
{600, 890},
{600, 539},
{602, 615},
{603, 583},
{604, 957},
{604, 593},
{606, 674},
{607, 636},
{608, 780},
{609, 733},
{610, 524},
{611, 491},
{612, 837},
{613, 802},
{614, 583},
{615, 535},
{616, 861},
{616, 516},
{618, 592},
{619, 551},
{620, 916},
{620, 561},
{622, 647},
{623, 602},
{624, 901},
{624, 548},
{626, 630},
{627, 587},
{628, 962},
{628, 598},
{630, 681},
{631, 638},
{632, 972},
{632, 606},
{634, 688},
{635, 649},
{636, 1038},
{636, 659},
{637, 620},
{639, 703},
{639, 385},
{640, 656},
{641, 622},
{643, 703},
{643, 384},
{644, 723},
{645, 686},
{647, 766},
{647, 436},
{648, 734},
{649, 694},
{651, 781},
{651, 449},
{652, 797},
{653, 756},
{654, 541},
{655, 505},
{656, 781},
{657, 741},
{658, 535},
{659, 496},
{660, 855},
{661, 809},
{662, 593},
{663, 549},
{664, 876},
{664, 528},
{666, 609},
{667, 568},
{668, 951},
{668, 589},
{670, 670},
{671, 635},
{672, 777},
{673, 735},
{674, 516},
{675, 483},
{676, 835},
{677, 792},
{678, 577},
{679, 538},
{680, 855},
{681, 804},
{682, 589},
{683, 552},
{684, 920},
{684, 564},
{686, 644},
{687, 602},
{688, 900},
{688, 547},
{690, 626},
{691, 588},
{692, 971},
{692, 605},
{694, 684},
{695, 646},
{696, 988},
{696, 619},
{698, 705},
{699, 667},
{699, 355},
{700, 680},
{701, 641},
{703, 731},
{704, 778},
{705, 737},
{706, 524},
{707, 492},
{708, 845},
{709, 809},
{710, 587},
{711, 559},
{712, 870},
{712, 523},
{714, 604},
{715, 565},
{716, 948},
{716, 587},
{718, 672},
{719, 630},
{720, 929},
{720, 571},
{722, 665},
{723, 622},
{724, 1012},
{724, 638},
{725, 601},
{727, 678},
{727, 364},
{728, 653},
{729, 616},
{731, 699},
{731, 381},
{732, 715},
{733, 674},
{735, 763},
{735, 434},
{736, 565},
{738, 647},
{739, 604},
{740, 998},
{740, 627},
{742, 712},
{743, 664},
{744, 1016},
{744, 641},
{745, 604},
{747, 683},
{747, 368},
{748, 703},
{749, 672},
{751, 745},
{751, 420},
{752, 701},
{753, 656},
{755, 750},
{755, 424},
{756, 762},
{757, 726},
{758, 518},
{759, 481},
{760, 794},
{761, 748},
{762, 545},
{763, 503},
{764, 869},
{764, 522},
{766, 605},
{767, 572},
{768, 972},
{768, 606},
{770, 687},
{771, 650},
{771, 340},
{772, 679},
{773, 636},
{775, 721},
{775, 399},
{776, 688},
{777, 651},
{779, 743},
{779, 418},
{780, 757},
{781, 717},
{783, 814},
{783, 477},
{784, 749},
{785, 714},
{787, 798},
{787, 464},
{788, 813},
{789, 777},
{790, 561},
{791, 525},
{792, 847},
{793, 792},
{794, 579},
{795, 543},
{796, 909},
{796, 555},
{798, 644},
{799, 600},
{800, 744},
{801, 701},
{803, 795},
{803, 461},
{804, 813},
{805, 769},
{806, 565},
{807, 523},
{808, 841},
{809, 801},
{810, 579},
{811, 546},
{812, 912},
{812, 557},
{814, 648},
{815, 602},
{816, 912},
{816, 557},
{818, 639},
{819, 595},
{820, 981},
{820, 613},
{822, 701},
{823, 664},
{824, 1011},
{824, 637},
{826, 721},
{827, 679},
{827, 364},
{828, 693},
{829, 652},
{831, 743},
{832, 790},
{833, 743},
{834, 531},
{835, 495},
{836, 855},
{837, 804},
{838, 595},
{839, 543},
{840, 872},
{840, 525},
{842, 604},
{843, 561},
{844, 940},
{844, 580},
{846, 656},
{847, 620},
{848, 917},
{848, 562},
{850, 641},
{851, 604},
{852, 988},
{852, 619},
{854, 704},
{855, 664},
{856, 1006},
{856, 633},
{858, 717},
{859, 674},
{859, 360},
{860, 693},
{861, 655},
{863, 738},
{864, 891},
{864, 540},
{866, 624},
{867, 581},
{868, 965},
{868, 601},
{870, 679},
{871, 641},
{872, 982},
{872, 614},
{874, 689},
{875, 648},
{876, 1042},
{876, 662},
{877, 623},
{879, 705},
{879, 386},
{880, 640},
{881, 599},
{883, 682},
{883, 367},
{884, 689},
{885, 645},
{887, 730},
{887, 407},
{888, 696},
{889, 649},
{891, 730},
{891, 407},
{892, 749},
{893, 700},
{895, 787},
{895, 454},
{896, 735},
{897, 697},
{899, 785},
{899, 452},
{900, 798},
{901, 744},
{902, 536},
{903, 496},
{904, 794},
{905, 745},
{906, 529},
{907, 496},
{908, 849},
{909, 795},
{910, 581},
{911, 541},
{912, 816},
{913, 769},
{914, 559},
{915, 509},
{916, 881},
{916, 531},
{918, 602},
{919, 556},
{920, 868},
{920, 521},
{922, 600},
{923, 548},
{924, 923},
{924, 566},
{926, 638},
{927, 601},
{928, 732},
{929, 693},
{931, 765},
{931, 436},
{932, 778},
{933, 723},
{934, 520},
{935, 474},
{936, 765},
{937, 728},
{939, 801},
{939, 466},
{940, 817},
{941, 764},
{942, 550},
{943, 505},
{944, 768},
{945, 728},
{947, 805},
{947, 469},
{948, 814},
{949, 773},
{950, 551},
{951, 508},
{952, 810},
{953, 770},
{954, 549},
{955, 510},
{956, 863},
{956, 517},
{958, 585},
{959, 543},
{960, 587},
{962, 659},
{963, 623},
{964, 996},
{964, 625},
{966, 699},
{967, 662},
{968, 993},
{968, 622},
{970, 692},
{971, 655},
{971, 344},
{972, 665},
{973, 609},
{975, 688},
{976, 997},
{976, 626},
{978, 698},
{979, 660},
{979, 348},
{980, 659},
{981, 614},
{983, 698},
{983, 380},
{984, 651},
{985, 617},
{987, 688},
{987, 372},
{988, 698},
{989, 643},
{991, 725},
{992, 862},
{993, 811},
{994, 595},
{995, 548},
{996, 904},
{996, 551},
{998, 629},
{999, 585},
{1000, 896},
{1000, 544},
{1002, 619},
{1003, 582},
{1004, 947},
{1004, 585},
{1006, 655},
{1007, 610},
{1008, 903},
{1008, 550},
{1010, 625},
{1011, 586},
{1012, 949},
{1012, 587},
{1014, 665},
{1015, 617},
{1016, 948},
{1016, 586},
{1018, 665},
{1019, 615},
{1020, 992},
{1020, 622},
{1022, 696},
{1023, 655},
};