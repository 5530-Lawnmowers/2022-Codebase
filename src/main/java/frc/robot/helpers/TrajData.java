package frc.robot.helpers;

import com.ctre.phoenix.motion.TrajectoryPoint;

import java.util.ArrayList;

public class TrajData {

    public double[] position =
            {0.0000000000,
                    0.0009702085,
                    0.0028868585,
                    0.0047678577,
                    0.0066132062,
                    0.0084229040,
                    0.0101969511,
                    0.0119353475,
                    0.0136380932,
                    0.0153051882,
                    0.0169366324,
                    0.0185324260,
                    0.0200925688,
                    0.0216170610,
                    0.0231059024,
                    0.0245590932,
                    0.0259766332,
                    0.0273585225,
                    0.0287047611,
                    0.0300153490,
                    0.0312902862,
                    0.0325295727,
                    0.0337332085,
                    0.0349011936,
                    0.0360335279,
                    0.0371302116,
                    0.0381912446,
                    0.0392166268,
                    0.0402063583,
                    0.0411604392,
                    0.0420788693,
                    0.0429616487,
                    0.0438087774,
                    0.0446202554,
                    0.0453960827,
                    0.0461362593,
                    0.0468407852,
                    0.0475096604,
                    0.0481428848,
                    0.0487404586,
                    0.0493023817,
                    0.0498286540,
                    0.0503192756,
                    0.0507742466,
                    0.0511935668,
                    0.0515772363,
                    0.0519252551,
                    0.0522376232,
                    0.0525143406,
                    0.0527554073,
                    0.0529608233,
                    0.0526185530,
                    0.0517621945,
                    0.0509199972,
                    0.0500920655,
                    0.0492785050,
                    0.0484794220,
                    0.0476949234,
                    0.0469251162,
                    0.0461701075,
                    0.0454300039,
                    0.0447049113,
                    0.0439949343,
                    0.0433001758,
                    0.0426207364,
                    0.0419567141,
                    0.0413082036,
                    0.0406752953,
                    0.0400580753,
                    0.0394566241,
                    0.0388710163,
                    0.0383013193,
                    0.0377475931,
                    0.0372098889,
                    0.0366882484,
                    0.0361827033,
                    0.0356932737,
                    0.0352199678,
                    0.0347627806,
                    0.0343216935,
                    0.0338966728,
                    0.0334876695,
                    0.0330946181,
                    0.0327174364,
                    0.0323560242,
                    0.0320102633,
                    0.0316800168,
                    0.0313651290,
                    0.0310654246,
                    0.0307807093,
                    0.0305107693,
                    0.0302553718,
                    0.0300142649,
                    0.0297871782,
                    0.0295738235,
                    0.0293738952,
                    0.0291870712,
                    0.0290130137,
                    0.0288513705,
                    0.0287017758,
                    0.0285638515,
                    0.0284372084,
                    0.0283214476,
                    0.0282161618,
                    0.0281209365,
                    0.0280353519,
                    0.0279589837,
                    0.0278914047,
                    0.0278321862,
                    0.0277808992,
                    0.0277371157,
                    0.0277004102,
                    0.0276703600,
                    0.0276465474,
                    0.0276285596,
                    0.0276159906,
                    0.0276084413,
                    0.0276055207,
                    0.0276068463,
                    0.0276120451,
                    0.0276207538,
                    0.0276326194,
                    0.0276472997,
                    0.0276644633,
                    0.0276837906,
                    0.0277049731,
                    0.0277277144,
                    0.0277517299,
                    0.0277767470,
                    0.0278025052,
                    0.0278287559,
                    0.0278552628,
                    0.0278818016,
                    0.0279081600,
                    0.0279341378,
                    0.0279595464,
                    0.0279842091,
                    0.0280079612,
                    0.0280306491,
                    0.0280521310,
                    0.0280722766,
                    0.0280909665,
                    0.0281080928,
                    0.0281235587,
                    0.0281372781,
                    0.0281491763,
                    0.0281591891,
                    0.0281672632,
                    0.0281733559,
                    0.0281774354,
                    0.0281794805,
                    0.0281794805,
                    0.0281774354,
                    0.0281733559,
                    0.0281672632,
                    0.0281591891,
                    0.0281491763,
                    0.0281372781,
                    0.0281235587,
                    0.0281080928,
                    0.0280909665,
                    0.0280722766,
                    0.0280521310,
                    0.0280306491,
                    0.0280079612,
                    0.0279842091,
                    0.0279595464,
                    0.0279341378,
                    0.0279081600,
                    0.0278818016,
                    0.0278552628,
                    0.0278287559,
                    0.0278025052,
                    0.0277767470,
                    0.0277517299,
                    0.0277277144,
                    0.0277049731,
                    0.0276837906,
                    0.0276644633,
                    0.0276472997,
                    0.0276326194,
                    0.0276207538,
                    0.0276120451,
                    0.0276068463,
                    0.0276055207,
                    0.0276084413,
                    0.0276159906,
                    0.0276285596,
                    0.0276465474,
                    0.0276703600,
                    0.0277004102,
                    0.0277371157,
                    0.0277808992,
                    0.0278321862,
                    0.0278914047,
                    0.0279589837,
                    0.0280353519,
                    0.0281209365,
                    0.0282161618,
                    0.0283214476,
                    0.0284372084,
                    0.0285638515,
                    0.0287017758,
                    0.0288513705,
                    0.0290130137,
                    0.0291870712,
                    0.0293738952,
                    0.0295738235,
                    0.0297871782,
                    0.0300142649,
                    0.0302553718,
                    0.0305107693,
                    0.0307807093,
                    0.0310654246,
                    0.0313651290,
                    0.0316800168,
                    0.0320102633,
                    0.0323560242,
                    0.0327174364,
                    0.0330946181,
                    0.0334876695,
                    0.0338966728,
                    0.0343216935,
                    0.0347627806,
                    0.0352199678,
                    0.0356932737,
                    0.0361827033,
                    0.0366882484,
                    0.0372098889,
                    0.0377475931,
                    0.0383013193,
                    0.0388710163,
                    0.0394566241,
                    0.0400580753,
                    0.0406752953,
                    0.0413082036,
                    0.0419567141,
                    0.0426207364,
                    0.0433001758,
                    0.0439949343,
                    0.0447049113,
                    0.0454300039,
                    0.0461701075,
                    0.0469251162,
                    0.0476949234,
                    0.0484794220,
                    0.0492785050,
                    0.0500920655,
                    0.0509199972,
                    0.0517621945,
                    0.0526185530};
    ArrayList Trajpoints = new ArrayList<TrajectoryPoint>();
    double[] velocity = {0,
            0.0970208533,
            0.1916649928,
            0.1880999221,
            0.1845348514,
            0.1809697806,
            0.1774047099,
            0.1738396392,
            0.1702745684,
            0.1667094977,
            0.163144427,
            0.1595793563,
            0.1560142855,
            0.1524492148,
            0.1488841441,
            0.1453190734,
            0.1417540026,
            0.1381889319,
            0.1346238612,
            0.1310587905,
            0.1274937197,
            0.123928649,
            0.1203635783,
            0.1167985076,
            0.1132334368,
            0.1096683661,
            0.1061032954,
            0.1025382247,
            0.0989731539,
            0.0954080832,
            0.0918430125,
            0.0882779418,
            0.084712871,
            0.0811478003,
            0.0775827296,
            0.0740176589,
            0.0704525881,
            0.0668875174,
            0.0633224467,
            0.059757376,
            0.0561923052,
            0.0526272345,
            0.0490621638,
            0.0454970931,
            0.0419320223,
            0.0383669516,
            0.0348018809,
            0.0312368102,
            0.0276717394,
            0.0241066687,
            0.020541598,
            -0.0342270248,
            -0.0856358482,
            -0.0842197331,
            -0.0827931678,
            -0.0813560502,
            -0.0799083011,
            -0.0784498664,
            -0.0769807204,
            -0.0755008695,
            -0.0740103552,
            -0.0725092578,
            -0.0709977006,
            -0.0694758535,
            -0.0679439371,
            -0.066402227,
            -0.0648510578,
            -0.0632908275,
            -0.0617220013,
            -0.0601451161,
            -0.0585607837,
            -0.0569696949,
            -0.0553726224,
            -0.0537704236,
            -0.0521640432,
            -0.0505545143,
            -0.0489429599,
            -0.0473305929,
            -0.045718715,
            -0.0441087159,
            -0.0425020695,
            -0.0409003312,
            -0.0393051322,
            -0.0377181737,
            -0.0361412199,
            -0.0345760895,
            -0.0330246462,
            -0.0314887885,
            -0.0299704386,
            -0.0284715299,
            -0.0269939951,
            -0.0255397523,
            -0.0241106926,
            -0.0227086662,
            -0.0213354692,
            -0.0199928313,
            -0.0186824033,
            -0.0174057457,
            -0.016164318,
            -0.0149594698,
            -0.0137924321,
            -0.0126643109,
            -0.0115760812,
            -0.0105285837,
            -0.0095225215,
            -0.0085584591,
            -0.007636823,
            -0.0067579027,
            -0.0059218534,
            -0.0051287001,
            -0.0043783419,
            -0.0036705578,
            -0.003005013,
            -0.0023812654,
            -0.0017987728,
            -0.0012569007,
            -0.0007549298,
            -0.0002920637,
            0.0001325637,
            0.0005198804,
            0.0008708688,
            0.0011865593,
            0.0014680232,
            0.0017163666,
            0.0019327241,
            0.002118254,
            0.0022741329,
            0.0024015512,
            0.0025017089,
            0.0025758121,
            0.0026250699,
            0.0026506909,
            0.0026538817,
            0.0026358441,
            0.0025977739,
            0.0025408589,
            0.0024662787,
            0.0023752031,
            0.002268792,
            0.0021481953,
            0.002014552,
            0.0018689914,
            0.0017126325,
            0.001546585,
            0.0013719496,
            0.0011898188,
            0.0010012778,
            0.0008074052,
            0.0006092741,
            0.0004079531,
            0.0002045074,
            0,
            -0.0002045074,
            -0.0004079531,
            -0.0006092741,
            -0.0008074052,
            -0.0010012778,
            -0.0011898188,
            -0.0013719496,
            -0.001546585,
            -0.0017126325,
            -0.0018689914,
            -0.002014552,
            -0.0021481953,
            -0.002268792,
            -0.0023752031,
            -0.0024662787,
            -0.0025408589,
            -0.0025977739,
            -0.0026358441,
            -0.0026538817,
            -0.0026506909,
            -0.0026250699,
            -0.0025758121,
            -0.0025017089,
            -0.0024015512,
            -0.0022741329,
            -0.002118254,
            -0.0019327241,
            -0.0017163666,
            -0.0014680232,
            -0.0011865593,
            -0.0008708688,
            -0.0005198804,
            -0.0001325637,
            0.0002920637,
            0.0007549298,
            0.0012569007,
            0.0017987728,
            0.0023812654,
            0.003005013,
            0.0036705578,
            0.0043783419,
            0.0051287001,
            0.0059218534,
            0.0067579027,
            0.007636823,
            0.0085584591,
            0.0095225215,
            0.0105285837,
            0.0115760812,
            0.0126643109,
            0.0137924321,
            0.0149594698,
            0.016164318,
            0.0174057457,
            0.0186824033,
            0.0199928313,
            0.0213354692,
            0.0227086662,
            0.0241106926,
            0.0255397523,
            0.0269939951,
            0.0284715299,
            0.0299704386,
            0.0314887885,
            0.0330246462,
            0.0345760895,
            0.0361412199,
            0.0377181737,
            0.0393051322,
            0.0409003312,
            0.0425020695,
            0.0441087159,
            0.045718715,
            0.0473305929,
            0.0489429599,
            0.0505545143,
            0.0521640432,
            0.0537704236,
            0.0553726224,
            0.0569696949,
            0.0585607837,
            0.0601451161,
            0.0617220013,
            0.0632908275,
            0.0648510578,
            0.066402227,
            0.0679439371,
            0.0694758535,
            0.0709977006,
            0.0725092578,
            0.0740103552,
            0.0755008695,
            0.0769807204,
            0.0784498664,
            0.0799083011,
            0.0813560502,
            0.0827931678,
            0.0842197331,
            0.0856358482};

    public ArrayList<TrajectoryPoint> data() {
        for (int i = 1; i < position.length; i++) {
            TrajectoryPoint n = new TrajectoryPoint();
            n.position = position[i - 1];
            n.velocity = velocity[i - 1];
            n.timeDur = 10;
            if (i == 1) {
                n.zeroPos = true;
            }
            Trajpoints.add(n);

        }
        return Trajpoints;
    }
}


