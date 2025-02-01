#ifndef WAVETABLE_TRIANGLE_H
#define WAVETABLE_TRIANGLE_H

#define WAVETABLE_SIZE 4096

const int16_t triangle_table[WAVETABLE_SIZE] = {
    -32767, -32735, -32703, -32671, -32639, -32607, -32575, -32543, 
    -32511, -32479, -32447, -32415, -32383, -32351, -32319, -32287, 
    -32255, -32223, -32191, -32159, -32127, -32095, -32063, -32031, 
    -31999, -31967, -31935, -31903, -31871, -31839, -31807, -31775, 
    -31743, -31711, -31679, -31647, -31615, -31583, -31551, -31519, 
    -31487, -31455, -31423, -31391, -31359, -31327, -31295, -31263, 
    -31231, -31199, -31167, -31135, -31103, -31071, -31039, -31007, 
    -30975, -30943, -30911, -30879, -30847, -30815, -30783, -30751, 
    -30719, -30687, -30655, -30623, -30591, -30559, -30527, -30495, 
    -30463, -30431, -30399, -30367, -30335, -30303, -30271, -30239, 
    -30207, -30175, -30143, -30111, -30079, -30047, -30015, -29983, 
    -29951, -29919, -29887, -29855, -29823, -29791, -29759, -29727, 
    -29695, -29663, -29631, -29599, -29567, -29535, -29503, -29471, 
    -29439, -29407, -29375, -29343, -29311, -29279, -29247, -29215, 
    -29183, -29151, -29119, -29087, -29055, -29023, -28991, -28959, 
    -28927, -28895, -28863, -28831, -28799, -28767, -28735, -28703, 
    -28671, -28639, -28607, -28575, -28543, -28511, -28479, -28447, 
    -28415, -28383, -28351, -28319, -28287, -28255, -28223, -28191, 
    -28159, -28127, -28095, -28063, -28031, -27999, -27967, -27935, 
    -27903, -27871, -27839, -27807, -27775, -27743, -27711, -27679, 
    -27647, -27615, -27583, -27551, -27519, -27487, -27455, -27423, 
    -27391, -27359, -27327, -27295, -27263, -27231, -27199, -27167, 
    -27135, -27103, -27071, -27039, -27007, -26975, -26943, -26911, 
    -26879, -26847, -26815, -26783, -26751, -26719, -26687, -26655, 
    -26623, -26591, -26559, -26527, -26495, -26463, -26431, -26399, 
    -26367, -26335, -26303, -26271, -26239, -26207, -26175, -26143, 
    -26111, -26079, -26047, -26015, -25983, -25951, -25919, -25887, 
    -25855, -25823, -25791, -25759, -25727, -25695, -25663, -25631, 
    -25599, -25567, -25535, -25503, -25471, -25439, -25407, -25375, 
    -25343, -25311, -25279, -25247, -25215, -25183, -25151, -25119, 
    -25087, -25055, -25023, -24991, -24959, -24927, -24895, -24863, 
    -24831, -24799, -24767, -24735, -24703, -24671, -24639, -24607, 
    -24575, -24543, -24511, -24479, -24447, -24415, -24383, -24351, 
    -24319, -24287, -24255, -24223, -24191, -24159, -24127, -24095, 
    -24063, -24031, -23999, -23967, -23935, -23903, -23871, -23839, 
    -23807, -23775, -23743, -23711, -23679, -23647, -23615, -23583, 
    -23551, -23519, -23487, -23455, -23423, -23391, -23359, -23327, 
    -23295, -23263, -23231, -23199, -23167, -23135, -23103, -23071, 
    -23039, -23007, -22975, -22943, -22911, -22879, -22847, -22815, 
    -22783, -22751, -22719, -22687, -22655, -22623, -22591, -22559, 
    -22527, -22495, -22463, -22431, -22399, -22367, -22335, -22303, 
    -22271, -22239, -22207, -22175, -22143, -22111, -22079, -22047, 
    -22015, -21983, -21951, -21919, -21887, -21855, -21823, -21791, 
    -21759, -21727, -21695, -21663, -21631, -21599, -21567, -21535, 
    -21503, -21471, -21439, -21407, -21375, -21343, -21311, -21279, 
    -21247, -21215, -21183, -21151, -21119, -21087, -21055, -21023, 
    -20991, -20959, -20927, -20895, -20863, -20831, -20799, -20767, 
    -20735, -20703, -20671, -20639, -20607, -20575, -20543, -20511, 
    -20479, -20447, -20415, -20383, -20351, -20319, -20287, -20255, 
    -20223, -20191, -20159, -20127, -20095, -20063, -20031, -19999, 
    -19967, -19935, -19903, -19871, -19839, -19807, -19775, -19743, 
    -19711, -19679, -19647, -19615, -19583, -19551, -19519, -19487, 
    -19455, -19423, -19391, -19359, -19327, -19295, -19263, -19231, 
    -19199, -19167, -19135, -19103, -19071, -19039, -19007, -18975, 
    -18943, -18911, -18879, -18847, -18815, -18783, -18751, -18719, 
    -18687, -18655, -18623, -18591, -18559, -18527, -18495, -18463, 
    -18431, -18399, -18367, -18335, -18303, -18271, -18239, -18207, 
    -18175, -18143, -18111, -18079, -18047, -18015, -17983, -17951, 
    -17919, -17887, -17855, -17823, -17791, -17759, -17727, -17695, 
    -17663, -17631, -17599, -17567, -17535, -17503, -17471, -17439, 
    -17407, -17375, -17343, -17311, -17279, -17247, -17215, -17183, 
    -17151, -17119, -17087, -17055, -17023, -16991, -16959, -16927, 
    -16895, -16863, -16831, -16799, -16767, -16735, -16703, -16671, 
    -16639, -16607, -16575, -16543, -16511, -16479, -16447, -16415, 
    -16383, -16351, -16319, -16287, -16255, -16223, -16191, -16159, 
    -16127, -16095, -16063, -16031, -15999, -15967, -15935, -15903, 
    -15871, -15839, -15807, -15775, -15743, -15711, -15679, -15647, 
    -15615, -15583, -15551, -15519, -15487, -15455, -15423, -15391, 
    -15359, -15327, -15295, -15263, -15231, -15199, -15167, -15135, 
    -15103, -15071, -15039, -15007, -14975, -14943, -14911, -14879, 
    -14847, -14815, -14783, -14751, -14719, -14687, -14655, -14623, 
    -14591, -14559, -14527, -14495, -14463, -14431, -14399, -14367, 
    -14335, -14303, -14271, -14239, -14207, -14175, -14143, -14111, 
    -14079, -14047, -14015, -13983, -13951, -13919, -13887, -13855, 
    -13823, -13791, -13759, -13727, -13695, -13663, -13631, -13599, 
    -13567, -13535, -13503, -13471, -13439, -13407, -13375, -13343, 
    -13311, -13279, -13247, -13215, -13183, -13151, -13119, -13087, 
    -13055, -13023, -12991, -12959, -12927, -12895, -12863, -12831, 
    -12799, -12767, -12735, -12703, -12671, -12639, -12607, -12575, 
    -12543, -12511, -12479, -12447, -12415, -12383, -12351, -12319, 
    -12287, -12255, -12223, -12191, -12159, -12127, -12095, -12063, 
    -12031, -11999, -11967, -11935, -11903, -11871, -11839, -11807, 
    -11775, -11743, -11711, -11679, -11647, -11615, -11583, -11551, 
    -11519, -11487, -11455, -11423, -11391, -11359, -11327, -11295, 
    -11263, -11231, -11199, -11167, -11135, -11103, -11071, -11039, 
    -11007, -10975, -10943, -10911, -10879, -10847, -10815, -10783, 
    -10751, -10719, -10687, -10655, -10623, -10591, -10559, -10527, 
    -10495, -10463, -10431, -10399, -10367, -10335, -10303, -10271, 
    -10239, -10207, -10175, -10143, -10111, -10079, -10047, -10015, 
    -9983, -9951, -9919, -9887, -9855, -9823, -9791, -9759, 
    -9727, -9695, -9663, -9631, -9599, -9567, -9535, -9503, 
    -9471, -9439, -9407, -9375, -9343, -9311, -9279, -9247, 
    -9215, -9183, -9151, -9119, -9087, -9055, -9023, -8991, 
    -8959, -8927, -8895, -8863, -8831, -8799, -8767, -8735, 
    -8703, -8671, -8639, -8607, -8575, -8543, -8511, -8479, 
    -8447, -8415, -8383, -8351, -8319, -8287, -8255, -8223, 
    -8191, -8159, -8127, -8095, -8063, -8031, -7999, -7967, 
    -7935, -7903, -7871, -7839, -7807, -7775, -7743, -7711, 
    -7679, -7647, -7615, -7583, -7551, -7519, -7487, -7455, 
    -7423, -7391, -7359, -7327, -7295, -7263, -7231, -7199, 
    -7167, -7135, -7103, -7071, -7039, -7007, -6975, -6943, 
    -6911, -6879, -6847, -6815, -6783, -6751, -6719, -6687, 
    -6655, -6623, -6591, -6559, -6527, -6495, -6463, -6431, 
    -6399, -6367, -6335, -6303, -6271, -6239, -6207, -6175, 
    -6143, -6111, -6079, -6047, -6015, -5983, -5951, -5919, 
    -5887, -5855, -5823, -5791, -5759, -5727, -5695, -5663, 
    -5631, -5599, -5567, -5535, -5503, -5471, -5439, -5407, 
    -5375, -5343, -5311, -5279, -5247, -5215, -5183, -5151, 
    -5119, -5087, -5055, -5023, -4991, -4959, -4927, -4895, 
    -4863, -4831, -4799, -4767, -4735, -4703, -4671, -4639, 
    -4607, -4575, -4543, -4511, -4479, -4447, -4415, -4383, 
    -4351, -4319, -4287, -4255, -4223, -4191, -4159, -4127, 
    -4095, -4063, -4031, -3999, -3967, -3935, -3903, -3871, 
    -3839, -3807, -3775, -3743, -3711, -3679, -3647, -3615, 
    -3583, -3551, -3519, -3487, -3455, -3423, -3391, -3359, 
    -3327, -3295, -3263, -3231, -3199, -3167, -3135, -3103, 
    -3071, -3039, -3007, -2975, -2943, -2911, -2879, -2847, 
    -2815, -2783, -2751, -2719, -2687, -2655, -2623, -2591, 
    -2559, -2527, -2495, -2463, -2431, -2399, -2367, -2335, 
    -2303, -2271, -2239, -2207, -2175, -2143, -2111, -2079, 
    -2047, -2015, -1983, -1951, -1919, -1887, -1855, -1823, 
    -1791, -1759, -1727, -1695, -1663, -1631, -1599, -1567, 
    -1535, -1503, -1471, -1439, -1407, -1375, -1343, -1311, 
    -1279, -1247, -1215, -1183, -1151, -1119, -1087, -1055, 
    -1023, -991, -959, -927, -895, -863, -831, -799, 
    -767, -735, -703, -671, -639, -607, -575, -543, 
    -511, -479, -447, -415, -383, -351, -319, -287, 
    -255, -223, -191, -159, -127, -95, -63, -31, 
    0, 31, 63, 95, 127, 159, 191, 223, 
    255, 287, 319, 351, 383, 415, 447, 479, 
    511, 543, 575, 607, 639, 671, 703, 735, 
    767, 799, 831, 863, 895, 927, 959, 991, 
    1023, 1055, 1087, 1119, 1151, 1183, 1215, 1247, 
    1279, 1311, 1343, 1375, 1407, 1439, 1471, 1503, 
    1535, 1567, 1599, 1631, 1663, 1695, 1727, 1759, 
    1791, 1823, 1855, 1887, 1919, 1951, 1983, 2015, 
    2047, 2079, 2111, 2143, 2175, 2207, 2239, 2271, 
    2303, 2335, 2367, 2399, 2431, 2463, 2495, 2527, 
    2559, 2591, 2623, 2655, 2687, 2719, 2751, 2783, 
    2815, 2847, 2879, 2911, 2943, 2975, 3007, 3039, 
    3071, 3103, 3135, 3167, 3199, 3231, 3263, 3295, 
    3327, 3359, 3391, 3423, 3455, 3487, 3519, 3551, 
    3583, 3615, 3647, 3679, 3711, 3743, 3775, 3807, 
    3839, 3871, 3903, 3935, 3967, 3999, 4031, 4063, 
    4095, 4127, 4159, 4191, 4223, 4255, 4287, 4319, 
    4351, 4383, 4415, 4447, 4479, 4511, 4543, 4575, 
    4607, 4639, 4671, 4703, 4735, 4767, 4799, 4831, 
    4863, 4895, 4927, 4959, 4991, 5023, 5055, 5087, 
    5119, 5151, 5183, 5215, 5247, 5279, 5311, 5343, 
    5375, 5407, 5439, 5471, 5503, 5535, 5567, 5599, 
    5631, 5663, 5695, 5727, 5759, 5791, 5823, 5855, 
    5887, 5919, 5951, 5983, 6015, 6047, 6079, 6111, 
    6143, 6175, 6207, 6239, 6271, 6303, 6335, 6367, 
    6399, 6431, 6463, 6495, 6527, 6559, 6591, 6623, 
    6655, 6687, 6719, 6751, 6783, 6815, 6847, 6879, 
    6911, 6943, 6975, 7007, 7039, 7071, 7103, 7135, 
    7167, 7199, 7231, 7263, 7295, 7327, 7359, 7391, 
    7423, 7455, 7487, 7519, 7551, 7583, 7615, 7647, 
    7679, 7711, 7743, 7775, 7807, 7839, 7871, 7903, 
    7935, 7967, 7999, 8031, 8063, 8095, 8127, 8159, 
    8191, 8223, 8255, 8287, 8319, 8351, 8383, 8415, 
    8447, 8479, 8511, 8543, 8575, 8607, 8639, 8671, 
    8703, 8735, 8767, 8799, 8831, 8863, 8895, 8927, 
    8959, 8991, 9023, 9055, 9087, 9119, 9151, 9183, 
    9215, 9247, 9279, 9311, 9343, 9375, 9407, 9439, 
    9471, 9503, 9535, 9567, 9599, 9631, 9663, 9695, 
    9727, 9759, 9791, 9823, 9855, 9887, 9919, 9951, 
    9983, 10015, 10047, 10079, 10111, 10143, 10175, 10207, 
    10239, 10271, 10303, 10335, 10367, 10399, 10431, 10463, 
    10495, 10527, 10559, 10591, 10623, 10655, 10687, 10719, 
    10751, 10783, 10815, 10847, 10879, 10911, 10943, 10975, 
    11007, 11039, 11071, 11103, 11135, 11167, 11199, 11231, 
    11263, 11295, 11327, 11359, 11391, 11423, 11455, 11487, 
    11519, 11551, 11583, 11615, 11647, 11679, 11711, 11743, 
    11775, 11807, 11839, 11871, 11903, 11935, 11967, 11999, 
    12031, 12063, 12095, 12127, 12159, 12191, 12223, 12255, 
    12287, 12319, 12351, 12383, 12415, 12447, 12479, 12511, 
    12543, 12575, 12607, 12639, 12671, 12703, 12735, 12767, 
    12799, 12831, 12863, 12895, 12927, 12959, 12991, 13023, 
    13055, 13087, 13119, 13151, 13183, 13215, 13247, 13279, 
    13311, 13343, 13375, 13407, 13439, 13471, 13503, 13535, 
    13567, 13599, 13631, 13663, 13695, 13727, 13759, 13791, 
    13823, 13855, 13887, 13919, 13951, 13983, 14015, 14047, 
    14079, 14111, 14143, 14175, 14207, 14239, 14271, 14303, 
    14335, 14367, 14399, 14431, 14463, 14495, 14527, 14559, 
    14591, 14623, 14655, 14687, 14719, 14751, 14783, 14815, 
    14847, 14879, 14911, 14943, 14975, 15007, 15039, 15071, 
    15103, 15135, 15167, 15199, 15231, 15263, 15295, 15327, 
    15359, 15391, 15423, 15455, 15487, 15519, 15551, 15583, 
    15615, 15647, 15679, 15711, 15743, 15775, 15807, 15839, 
    15871, 15903, 15935, 15967, 15999, 16031, 16063, 16095, 
    16127, 16159, 16191, 16223, 16255, 16287, 16319, 16351, 
    16383, 16415, 16447, 16479, 16511, 16543, 16575, 16607, 
    16639, 16671, 16703, 16735, 16767, 16799, 16831, 16863, 
    16895, 16927, 16959, 16991, 17023, 17055, 17087, 17119, 
    17151, 17183, 17215, 17247, 17279, 17311, 17343, 17375, 
    17407, 17439, 17471, 17503, 17535, 17567, 17599, 17631, 
    17663, 17695, 17727, 17759, 17791, 17823, 17855, 17887, 
    17919, 17951, 17983, 18015, 18047, 18079, 18111, 18143, 
    18175, 18207, 18239, 18271, 18303, 18335, 18367, 18399, 
    18431, 18463, 18495, 18527, 18559, 18591, 18623, 18655, 
    18687, 18719, 18751, 18783, 18815, 18847, 18879, 18911, 
    18943, 18975, 19007, 19039, 19071, 19103, 19135, 19167, 
    19199, 19231, 19263, 19295, 19327, 19359, 19391, 19423, 
    19455, 19487, 19519, 19551, 19583, 19615, 19647, 19679, 
    19711, 19743, 19775, 19807, 19839, 19871, 19903, 19935, 
    19967, 19999, 20031, 20063, 20095, 20127, 20159, 20191, 
    20223, 20255, 20287, 20319, 20351, 20383, 20415, 20447, 
    20479, 20511, 20543, 20575, 20607, 20639, 20671, 20703, 
    20735, 20767, 20799, 20831, 20863, 20895, 20927, 20959, 
    20991, 21023, 21055, 21087, 21119, 21151, 21183, 21215, 
    21247, 21279, 21311, 21343, 21375, 21407, 21439, 21471, 
    21503, 21535, 21567, 21599, 21631, 21663, 21695, 21727, 
    21759, 21791, 21823, 21855, 21887, 21919, 21951, 21983, 
    22015, 22047, 22079, 22111, 22143, 22175, 22207, 22239, 
    22271, 22303, 22335, 22367, 22399, 22431, 22463, 22495, 
    22527, 22559, 22591, 22623, 22655, 22687, 22719, 22751, 
    22783, 22815, 22847, 22879, 22911, 22943, 22975, 23007, 
    23039, 23071, 23103, 23135, 23167, 23199, 23231, 23263, 
    23295, 23327, 23359, 23391, 23423, 23455, 23487, 23519, 
    23551, 23583, 23615, 23647, 23679, 23711, 23743, 23775, 
    23807, 23839, 23871, 23903, 23935, 23967, 23999, 24031, 
    24063, 24095, 24127, 24159, 24191, 24223, 24255, 24287, 
    24319, 24351, 24383, 24415, 24447, 24479, 24511, 24543, 
    24575, 24607, 24639, 24671, 24703, 24735, 24767, 24799, 
    24831, 24863, 24895, 24927, 24959, 24991, 25023, 25055, 
    25087, 25119, 25151, 25183, 25215, 25247, 25279, 25311, 
    25343, 25375, 25407, 25439, 25471, 25503, 25535, 25567, 
    25599, 25631, 25663, 25695, 25727, 25759, 25791, 25823, 
    25855, 25887, 25919, 25951, 25983, 26015, 26047, 26079, 
    26111, 26143, 26175, 26207, 26239, 26271, 26303, 26335, 
    26367, 26399, 26431, 26463, 26495, 26527, 26559, 26591, 
    26623, 26655, 26687, 26719, 26751, 26783, 26815, 26847, 
    26879, 26911, 26943, 26975, 27007, 27039, 27071, 27103, 
    27135, 27167, 27199, 27231, 27263, 27295, 27327, 27359, 
    27391, 27423, 27455, 27487, 27519, 27551, 27583, 27615, 
    27647, 27679, 27711, 27743, 27775, 27807, 27839, 27871, 
    27903, 27935, 27967, 27999, 28031, 28063, 28095, 28127, 
    28159, 28191, 28223, 28255, 28287, 28319, 28351, 28383, 
    28415, 28447, 28479, 28511, 28543, 28575, 28607, 28639, 
    28671, 28703, 28735, 28767, 28799, 28831, 28863, 28895, 
    28927, 28959, 28991, 29023, 29055, 29087, 29119, 29151, 
    29183, 29215, 29247, 29279, 29311, 29343, 29375, 29407, 
    29439, 29471, 29503, 29535, 29567, 29599, 29631, 29663, 
    29695, 29727, 29759, 29791, 29823, 29855, 29887, 29919, 
    29951, 29983, 30015, 30047, 30079, 30111, 30143, 30175, 
    30207, 30239, 30271, 30303, 30335, 30367, 30399, 30431, 
    30463, 30495, 30527, 30559, 30591, 30623, 30655, 30687, 
    30719, 30751, 30783, 30815, 30847, 30879, 30911, 30943, 
    30975, 31007, 31039, 31071, 31103, 31135, 31167, 31199, 
    31231, 31263, 31295, 31327, 31359, 31391, 31423, 31455, 
    31487, 31519, 31551, 31583, 31615, 31647, 31679, 31711, 
    31743, 31775, 31807, 31839, 31871, 31903, 31935, 31967, 
    31999, 32031, 32063, 32095, 32127, 32159, 32191, 32223, 
    32255, 32287, 32319, 32351, 32383, 32415, 32447, 32479, 
    32511, 32543, 32575, 32607, 32639, 32671, 32703, 32735, 
    32767, 32735, 32703, 32671, 32639, 32607, 32575, 32543, 
    32511, 32479, 32447, 32415, 32383, 32351, 32319, 32287, 
    32255, 32223, 32191, 32159, 32127, 32095, 32063, 32031, 
    31999, 31967, 31935, 31903, 31871, 31839, 31807, 31775, 
    31743, 31711, 31679, 31647, 31615, 31583, 31551, 31519, 
    31487, 31455, 31423, 31391, 31359, 31327, 31295, 31263, 
    31231, 31199, 31167, 31135, 31103, 31071, 31039, 31007, 
    30975, 30943, 30911, 30879, 30847, 30815, 30783, 30751, 
    30719, 30687, 30655, 30623, 30591, 30559, 30527, 30495, 
    30463, 30431, 30399, 30367, 30335, 30303, 30271, 30239, 
    30207, 30175, 30143, 30111, 30079, 30047, 30015, 29983, 
    29951, 29919, 29887, 29855, 29823, 29791, 29759, 29727, 
    29695, 29663, 29631, 29599, 29567, 29535, 29503, 29471, 
    29439, 29407, 29375, 29343, 29311, 29279, 29247, 29215, 
    29183, 29151, 29119, 29087, 29055, 29023, 28991, 28959, 
    28927, 28895, 28863, 28831, 28799, 28767, 28735, 28703, 
    28671, 28639, 28607, 28575, 28543, 28511, 28479, 28447, 
    28415, 28383, 28351, 28319, 28287, 28255, 28223, 28191, 
    28159, 28127, 28095, 28063, 28031, 27999, 27967, 27935, 
    27903, 27871, 27839, 27807, 27775, 27743, 27711, 27679, 
    27647, 27615, 27583, 27551, 27519, 27487, 27455, 27423, 
    27391, 27359, 27327, 27295, 27263, 27231, 27199, 27167, 
    27135, 27103, 27071, 27039, 27007, 26975, 26943, 26911, 
    26879, 26847, 26815, 26783, 26751, 26719, 26687, 26655, 
    26623, 26591, 26559, 26527, 26495, 26463, 26431, 26399, 
    26367, 26335, 26303, 26271, 26239, 26207, 26175, 26143, 
    26111, 26079, 26047, 26015, 25983, 25951, 25919, 25887, 
    25855, 25823, 25791, 25759, 25727, 25695, 25663, 25631, 
    25599, 25567, 25535, 25503, 25471, 25439, 25407, 25375, 
    25343, 25311, 25279, 25247, 25215, 25183, 25151, 25119, 
    25087, 25055, 25023, 24991, 24959, 24927, 24895, 24863, 
    24831, 24799, 24767, 24735, 24703, 24671, 24639, 24607, 
    24575, 24543, 24511, 24479, 24447, 24415, 24383, 24351, 
    24319, 24287, 24255, 24223, 24191, 24159, 24127, 24095, 
    24063, 24031, 23999, 23967, 23935, 23903, 23871, 23839, 
    23807, 23775, 23743, 23711, 23679, 23647, 23615, 23583, 
    23551, 23519, 23487, 23455, 23423, 23391, 23359, 23327, 
    23295, 23263, 23231, 23199, 23167, 23135, 23103, 23071, 
    23039, 23007, 22975, 22943, 22911, 22879, 22847, 22815, 
    22783, 22751, 22719, 22687, 22655, 22623, 22591, 22559, 
    22527, 22495, 22463, 22431, 22399, 22367, 22335, 22303, 
    22271, 22239, 22207, 22175, 22143, 22111, 22079, 22047, 
    22015, 21983, 21951, 21919, 21887, 21855, 21823, 21791, 
    21759, 21727, 21695, 21663, 21631, 21599, 21567, 21535, 
    21503, 21471, 21439, 21407, 21375, 21343, 21311, 21279, 
    21247, 21215, 21183, 21151, 21119, 21087, 21055, 21023, 
    20991, 20959, 20927, 20895, 20863, 20831, 20799, 20767, 
    20735, 20703, 20671, 20639, 20607, 20575, 20543, 20511, 
    20479, 20447, 20415, 20383, 20351, 20319, 20287, 20255, 
    20223, 20191, 20159, 20127, 20095, 20063, 20031, 19999, 
    19967, 19935, 19903, 19871, 19839, 19807, 19775, 19743, 
    19711, 19679, 19647, 19615, 19583, 19551, 19519, 19487, 
    19455, 19423, 19391, 19359, 19327, 19295, 19263, 19231, 
    19199, 19167, 19135, 19103, 19071, 19039, 19007, 18975, 
    18943, 18911, 18879, 18847, 18815, 18783, 18751, 18719, 
    18687, 18655, 18623, 18591, 18559, 18527, 18495, 18463, 
    18431, 18399, 18367, 18335, 18303, 18271, 18239, 18207, 
    18175, 18143, 18111, 18079, 18047, 18015, 17983, 17951, 
    17919, 17887, 17855, 17823, 17791, 17759, 17727, 17695, 
    17663, 17631, 17599, 17567, 17535, 17503, 17471, 17439, 
    17407, 17375, 17343, 17311, 17279, 17247, 17215, 17183, 
    17151, 17119, 17087, 17055, 17023, 16991, 16959, 16927, 
    16895, 16863, 16831, 16799, 16767, 16735, 16703, 16671, 
    16639, 16607, 16575, 16543, 16511, 16479, 16447, 16415, 
    16383, 16351, 16319, 16287, 16255, 16223, 16191, 16159, 
    16127, 16095, 16063, 16031, 15999, 15967, 15935, 15903, 
    15871, 15839, 15807, 15775, 15743, 15711, 15679, 15647, 
    15615, 15583, 15551, 15519, 15487, 15455, 15423, 15391, 
    15359, 15327, 15295, 15263, 15231, 15199, 15167, 15135, 
    15103, 15071, 15039, 15007, 14975, 14943, 14911, 14879, 
    14847, 14815, 14783, 14751, 14719, 14687, 14655, 14623, 
    14591, 14559, 14527, 14495, 14463, 14431, 14399, 14367, 
    14335, 14303, 14271, 14239, 14207, 14175, 14143, 14111, 
    14079, 14047, 14015, 13983, 13951, 13919, 13887, 13855, 
    13823, 13791, 13759, 13727, 13695, 13663, 13631, 13599, 
    13567, 13535, 13503, 13471, 13439, 13407, 13375, 13343, 
    13311, 13279, 13247, 13215, 13183, 13151, 13119, 13087, 
    13055, 13023, 12991, 12959, 12927, 12895, 12863, 12831, 
    12799, 12767, 12735, 12703, 12671, 12639, 12607, 12575, 
    12543, 12511, 12479, 12447, 12415, 12383, 12351, 12319, 
    12287, 12255, 12223, 12191, 12159, 12127, 12095, 12063, 
    12031, 11999, 11967, 11935, 11903, 11871, 11839, 11807, 
    11775, 11743, 11711, 11679, 11647, 11615, 11583, 11551, 
    11519, 11487, 11455, 11423, 11391, 11359, 11327, 11295, 
    11263, 11231, 11199, 11167, 11135, 11103, 11071, 11039, 
    11007, 10975, 10943, 10911, 10879, 10847, 10815, 10783, 
    10751, 10719, 10687, 10655, 10623, 10591, 10559, 10527, 
    10495, 10463, 10431, 10399, 10367, 10335, 10303, 10271, 
    10239, 10207, 10175, 10143, 10111, 10079, 10047, 10015, 
    9983, 9951, 9919, 9887, 9855, 9823, 9791, 9759, 
    9727, 9695, 9663, 9631, 9599, 9567, 9535, 9503, 
    9471, 9439, 9407, 9375, 9343, 9311, 9279, 9247, 
    9215, 9183, 9151, 9119, 9087, 9055, 9023, 8991, 
    8959, 8927, 8895, 8863, 8831, 8799, 8767, 8735, 
    8703, 8671, 8639, 8607, 8575, 8543, 8511, 8479, 
    8447, 8415, 8383, 8351, 8319, 8287, 8255, 8223, 
    8191, 8159, 8127, 8095, 8063, 8031, 7999, 7967, 
    7935, 7903, 7871, 7839, 7807, 7775, 7743, 7711, 
    7679, 7647, 7615, 7583, 7551, 7519, 7487, 7455, 
    7423, 7391, 7359, 7327, 7295, 7263, 7231, 7199, 
    7167, 7135, 7103, 7071, 7039, 7007, 6975, 6943, 
    6911, 6879, 6847, 6815, 6783, 6751, 6719, 6687, 
    6655, 6623, 6591, 6559, 6527, 6495, 6463, 6431, 
    6399, 6367, 6335, 6303, 6271, 6239, 6207, 6175, 
    6143, 6111, 6079, 6047, 6015, 5983, 5951, 5919, 
    5887, 5855, 5823, 5791, 5759, 5727, 5695, 5663, 
    5631, 5599, 5567, 5535, 5503, 5471, 5439, 5407, 
    5375, 5343, 5311, 5279, 5247, 5215, 5183, 5151, 
    5119, 5087, 5055, 5023, 4991, 4959, 4927, 4895, 
    4863, 4831, 4799, 4767, 4735, 4703, 4671, 4639, 
    4607, 4575, 4543, 4511, 4479, 4447, 4415, 4383, 
    4351, 4319, 4287, 4255, 4223, 4191, 4159, 4127, 
    4095, 4063, 4031, 3999, 3967, 3935, 3903, 3871, 
    3839, 3807, 3775, 3743, 3711, 3679, 3647, 3615, 
    3583, 3551, 3519, 3487, 3455, 3423, 3391, 3359, 
    3327, 3295, 3263, 3231, 3199, 3167, 3135, 3103, 
    3071, 3039, 3007, 2975, 2943, 2911, 2879, 2847, 
    2815, 2783, 2751, 2719, 2687, 2655, 2623, 2591, 
    2559, 2527, 2495, 2463, 2431, 2399, 2367, 2335, 
    2303, 2271, 2239, 2207, 2175, 2143, 2111, 2079, 
    2047, 2015, 1983, 1951, 1919, 1887, 1855, 1823, 
    1791, 1759, 1727, 1695, 1663, 1631, 1599, 1567, 
    1535, 1503, 1471, 1439, 1407, 1375, 1343, 1311, 
    1279, 1247, 1215, 1183, 1151, 1119, 1087, 1055, 
    1023, 991, 959, 927, 895, 863, 831, 799, 
    767, 735, 703, 671, 639, 607, 575, 543, 
    511, 479, 447, 415, 383, 351, 319, 287, 
    255, 223, 191, 159, 127, 95, 63, 31, 
    0, -31, -63, -95, -127, -159, -191, -223, 
    -255, -287, -319, -351, -383, -415, -447, -479, 
    -511, -543, -575, -607, -639, -671, -703, -735, 
    -767, -799, -831, -863, -895, -927, -959, -991, 
    -1023, -1055, -1087, -1119, -1151, -1183, -1215, -1247, 
    -1279, -1311, -1343, -1375, -1407, -1439, -1471, -1503, 
    -1535, -1567, -1599, -1631, -1663, -1695, -1727, -1759, 
    -1791, -1823, -1855, -1887, -1919, -1951, -1983, -2015, 
    -2047, -2079, -2111, -2143, -2175, -2207, -2239, -2271, 
    -2303, -2335, -2367, -2399, -2431, -2463, -2495, -2527, 
    -2559, -2591, -2623, -2655, -2687, -2719, -2751, -2783, 
    -2815, -2847, -2879, -2911, -2943, -2975, -3007, -3039, 
    -3071, -3103, -3135, -3167, -3199, -3231, -3263, -3295, 
    -3327, -3359, -3391, -3423, -3455, -3487, -3519, -3551, 
    -3583, -3615, -3647, -3679, -3711, -3743, -3775, -3807, 
    -3839, -3871, -3903, -3935, -3967, -3999, -4031, -4063, 
    -4095, -4127, -4159, -4191, -4223, -4255, -4287, -4319, 
    -4351, -4383, -4415, -4447, -4479, -4511, -4543, -4575, 
    -4607, -4639, -4671, -4703, -4735, -4767, -4799, -4831, 
    -4863, -4895, -4927, -4959, -4991, -5023, -5055, -5087, 
    -5119, -5151, -5183, -5215, -5247, -5279, -5311, -5343, 
    -5375, -5407, -5439, -5471, -5503, -5535, -5567, -5599, 
    -5631, -5663, -5695, -5727, -5759, -5791, -5823, -5855, 
    -5887, -5919, -5951, -5983, -6015, -6047, -6079, -6111, 
    -6143, -6175, -6207, -6239, -6271, -6303, -6335, -6367, 
    -6399, -6431, -6463, -6495, -6527, -6559, -6591, -6623, 
    -6655, -6687, -6719, -6751, -6783, -6815, -6847, -6879, 
    -6911, -6943, -6975, -7007, -7039, -7071, -7103, -7135, 
    -7167, -7199, -7231, -7263, -7295, -7327, -7359, -7391, 
    -7423, -7455, -7487, -7519, -7551, -7583, -7615, -7647, 
    -7679, -7711, -7743, -7775, -7807, -7839, -7871, -7903, 
    -7935, -7967, -7999, -8031, -8063, -8095, -8127, -8159, 
    -8191, -8223, -8255, -8287, -8319, -8351, -8383, -8415, 
    -8447, -8479, -8511, -8543, -8575, -8607, -8639, -8671, 
    -8703, -8735, -8767, -8799, -8831, -8863, -8895, -8927, 
    -8959, -8991, -9023, -9055, -9087, -9119, -9151, -9183, 
    -9215, -9247, -9279, -9311, -9343, -9375, -9407, -9439, 
    -9471, -9503, -9535, -9567, -9599, -9631, -9663, -9695, 
    -9727, -9759, -9791, -9823, -9855, -9887, -9919, -9951, 
    -9983, -10015, -10047, -10079, -10111, -10143, -10175, -10207, 
    -10239, -10271, -10303, -10335, -10367, -10399, -10431, -10463, 
    -10495, -10527, -10559, -10591, -10623, -10655, -10687, -10719, 
    -10751, -10783, -10815, -10847, -10879, -10911, -10943, -10975, 
    -11007, -11039, -11071, -11103, -11135, -11167, -11199, -11231, 
    -11263, -11295, -11327, -11359, -11391, -11423, -11455, -11487, 
    -11519, -11551, -11583, -11615, -11647, -11679, -11711, -11743, 
    -11775, -11807, -11839, -11871, -11903, -11935, -11967, -11999, 
    -12031, -12063, -12095, -12127, -12159, -12191, -12223, -12255, 
    -12287, -12319, -12351, -12383, -12415, -12447, -12479, -12511, 
    -12543, -12575, -12607, -12639, -12671, -12703, -12735, -12767, 
    -12799, -12831, -12863, -12895, -12927, -12959, -12991, -13023, 
    -13055, -13087, -13119, -13151, -13183, -13215, -13247, -13279, 
    -13311, -13343, -13375, -13407, -13439, -13471, -13503, -13535, 
    -13567, -13599, -13631, -13663, -13695, -13727, -13759, -13791, 
    -13823, -13855, -13887, -13919, -13951, -13983, -14015, -14047, 
    -14079, -14111, -14143, -14175, -14207, -14239, -14271, -14303, 
    -14335, -14367, -14399, -14431, -14463, -14495, -14527, -14559, 
    -14591, -14623, -14655, -14687, -14719, -14751, -14783, -14815, 
    -14847, -14879, -14911, -14943, -14975, -15007, -15039, -15071, 
    -15103, -15135, -15167, -15199, -15231, -15263, -15295, -15327, 
    -15359, -15391, -15423, -15455, -15487, -15519, -15551, -15583, 
    -15615, -15647, -15679, -15711, -15743, -15775, -15807, -15839, 
    -15871, -15903, -15935, -15967, -15999, -16031, -16063, -16095, 
    -16127, -16159, -16191, -16223, -16255, -16287, -16319, -16351, 
    -16383, -16415, -16447, -16479, -16511, -16543, -16575, -16607, 
    -16639, -16671, -16703, -16735, -16767, -16799, -16831, -16863, 
    -16895, -16927, -16959, -16991, -17023, -17055, -17087, -17119, 
    -17151, -17183, -17215, -17247, -17279, -17311, -17343, -17375, 
    -17407, -17439, -17471, -17503, -17535, -17567, -17599, -17631, 
    -17663, -17695, -17727, -17759, -17791, -17823, -17855, -17887, 
    -17919, -17951, -17983, -18015, -18047, -18079, -18111, -18143, 
    -18175, -18207, -18239, -18271, -18303, -18335, -18367, -18399, 
    -18431, -18463, -18495, -18527, -18559, -18591, -18623, -18655, 
    -18687, -18719, -18751, -18783, -18815, -18847, -18879, -18911, 
    -18943, -18975, -19007, -19039, -19071, -19103, -19135, -19167, 
    -19199, -19231, -19263, -19295, -19327, -19359, -19391, -19423, 
    -19455, -19487, -19519, -19551, -19583, -19615, -19647, -19679, 
    -19711, -19743, -19775, -19807, -19839, -19871, -19903, -19935, 
    -19967, -19999, -20031, -20063, -20095, -20127, -20159, -20191, 
    -20223, -20255, -20287, -20319, -20351, -20383, -20415, -20447, 
    -20479, -20511, -20543, -20575, -20607, -20639, -20671, -20703, 
    -20735, -20767, -20799, -20831, -20863, -20895, -20927, -20959, 
    -20991, -21023, -21055, -21087, -21119, -21151, -21183, -21215, 
    -21247, -21279, -21311, -21343, -21375, -21407, -21439, -21471, 
    -21503, -21535, -21567, -21599, -21631, -21663, -21695, -21727, 
    -21759, -21791, -21823, -21855, -21887, -21919, -21951, -21983, 
    -22015, -22047, -22079, -22111, -22143, -22175, -22207, -22239, 
    -22271, -22303, -22335, -22367, -22399, -22431, -22463, -22495, 
    -22527, -22559, -22591, -22623, -22655, -22687, -22719, -22751, 
    -22783, -22815, -22847, -22879, -22911, -22943, -22975, -23007, 
    -23039, -23071, -23103, -23135, -23167, -23199, -23231, -23263, 
    -23295, -23327, -23359, -23391, -23423, -23455, -23487, -23519, 
    -23551, -23583, -23615, -23647, -23679, -23711, -23743, -23775, 
    -23807, -23839, -23871, -23903, -23935, -23967, -23999, -24031, 
    -24063, -24095, -24127, -24159, -24191, -24223, -24255, -24287, 
    -24319, -24351, -24383, -24415, -24447, -24479, -24511, -24543, 
    -24575, -24607, -24639, -24671, -24703, -24735, -24767, -24799, 
    -24831, -24863, -24895, -24927, -24959, -24991, -25023, -25055, 
    -25087, -25119, -25151, -25183, -25215, -25247, -25279, -25311, 
    -25343, -25375, -25407, -25439, -25471, -25503, -25535, -25567, 
    -25599, -25631, -25663, -25695, -25727, -25759, -25791, -25823, 
    -25855, -25887, -25919, -25951, -25983, -26015, -26047, -26079, 
    -26111, -26143, -26175, -26207, -26239, -26271, -26303, -26335, 
    -26367, -26399, -26431, -26463, -26495, -26527, -26559, -26591, 
    -26623, -26655, -26687, -26719, -26751, -26783, -26815, -26847, 
    -26879, -26911, -26943, -26975, -27007, -27039, -27071, -27103, 
    -27135, -27167, -27199, -27231, -27263, -27295, -27327, -27359, 
    -27391, -27423, -27455, -27487, -27519, -27551, -27583, -27615, 
    -27647, -27679, -27711, -27743, -27775, -27807, -27839, -27871, 
    -27903, -27935, -27967, -27999, -28031, -28063, -28095, -28127, 
    -28159, -28191, -28223, -28255, -28287, -28319, -28351, -28383, 
    -28415, -28447, -28479, -28511, -28543, -28575, -28607, -28639, 
    -28671, -28703, -28735, -28767, -28799, -28831, -28863, -28895, 
    -28927, -28959, -28991, -29023, -29055, -29087, -29119, -29151, 
    -29183, -29215, -29247, -29279, -29311, -29343, -29375, -29407, 
    -29439, -29471, -29503, -29535, -29567, -29599, -29631, -29663, 
    -29695, -29727, -29759, -29791, -29823, -29855, -29887, -29919, 
    -29951, -29983, -30015, -30047, -30079, -30111, -30143, -30175, 
    -30207, -30239, -30271, -30303, -30335, -30367, -30399, -30431, 
    -30463, -30495, -30527, -30559, -30591, -30623, -30655, -30687, 
    -30719, -30751, -30783, -30815, -30847, -30879, -30911, -30943, 
    -30975, -31007, -31039, -31071, -31103, -31135, -31167, -31199, 
    -31231, -31263, -31295, -31327, -31359, -31391, -31423, -31455, 
    -31487, -31519, -31551, -31583, -31615, -31647, -31679, -31711, 
    -31743, -31775, -31807, -31839, -31871, -31903, -31935, -31967, 
    -31999, -32031, -32063, -32095, -32127, -32159, -32191, -32223, 
    -32255, -32287, -32319, -32351, -32383, -32415, -32447, -32479, 
    -32511, -32543, -32575, -32607, -32639, -32671, -32703, -32735
    
};

#endif