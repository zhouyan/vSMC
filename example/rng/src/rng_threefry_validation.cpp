//============================================================================
// vSMC/example/rng/src/rng_threefry_validation.cpp
//----------------------------------------------------------------------------
//                         vSMC: Scalable Monte Carlo
//----------------------------------------------------------------------------
// Copyright (c) 2013-2015, Yan Zhou
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//============================================================================

#include "rng_validation.hpp"
#include <vsmc/rng/threefry.hpp>

int main()
{
    std::vector<unsigned long long> Threefry2x32Result = {941278031ULL,
        46291001ULL, 1214697033ULL, 1271548499ULL, 2816446740ULL,
        3702319852ULL, 3546019119ULL, 4126972354ULL, 3355327511ULL,
        2561075038ULL, 2867864609ULL, 1380192199ULL, 2646986690ULL,
        2976689751ULL, 2493325877ULL, 1597872593ULL, 4138635897ULL,
        1514800382ULL, 747037485ULL, 1051378934ULL, 3692487776ULL,
        1772478623ULL, 2339217922ULL, 1815813640ULL, 928985437ULL,
        1030728408ULL, 2217204681ULL, 2352029856ULL, 1472425253ULL,
        3425833415ULL, 102874842ULL, 806667352ULL, 3921031613ULL,
        1963471130ULL, 3353999392ULL, 3945376082ULL, 1244419834ULL,
        1166650210ULL, 855804086ULL, 2239135374ULL, 2340207573ULL,
        759573632ULL, 474348249ULL, 2948285509ULL, 3564767714ULL,
        1700091432ULL, 3369522284ULL, 1055985750ULL, 1886329525ULL,
        2540261158ULL, 4134431824ULL, 3495332379ULL, 814651382ULL,
        471922180ULL, 1389750517ULL, 3509641691ULL, 1048611836ULL,
        1200391414ULL, 3114936612ULL, 4125503531ULL, 3148248337ULL,
        2217658390ULL, 1411500667ULL, 16534989ULL, 2971789451ULL,
        2760509152ULL, 786211067ULL, 616060958ULL, 2307331502ULL,
        1656805867ULL, 2510980958ULL, 1360786839ULL, 2588668683ULL,
        2272944488ULL, 2472154675ULL, 2261488046ULL, 1601519537ULL,
        1576569540ULL, 968560149ULL, 1469563241ULL, 3659411079ULL,
        401638098ULL, 1189816362ULL, 1066675820ULL, 189870202ULL,
        3149003634ULL, 4013251633ULL, 2624973634ULL, 3071399113ULL,
        489392195ULL, 569170260ULL, 3151774943ULL, 1323299256ULL,
        3482885113ULL, 402439219ULL, 3163273511ULL, 3505379617ULL,
        1170217625ULL, 1181892006ULL, 970586880ULL, 92ULL};
    rng_validation<vsmc::Threefry2x32>(
        Threefry2x32Result, "Threefry2x32");

    std::vector<unsigned long long> Threefry4x32Result = {72649349ULL,
        488426255ULL, 1182236606ULL, 503389643ULL, 2657065516ULL,
        578980270ULL, 1627626910ULL, 169765934ULL, 1836901923ULL,
        2396129618ULL, 2501430438ULL, 1981230118ULL, 1374392207ULL,
        346584804ULL, 1314475032ULL, 2865600010ULL, 2570826903ULL,
        2569213752ULL, 3068642772ULL, 3722132040ULL, 375104652ULL,
        2674286333ULL, 4092951981ULL, 2592576854ULL, 2357214702ULL,
        1954129609ULL, 1603272422ULL, 2786787178ULL, 194198825ULL,
        4285936463ULL, 643234243ULL, 2453476840ULL, 3462240126ULL,
        3304269618ULL, 2786274927ULL, 3126421056ULL, 3519787310ULL,
        983455245ULL, 1762162433ULL, 4064380435ULL, 3671115716ULL,
        1974878152ULL, 247415759ULL, 2480779732ULL, 2071146543ULL,
        2559365569ULL, 2305712623ULL, 2007930127ULL, 470410896ULL,
        1481906799ULL, 1710719171ULL, 2452027994ULL, 1766532660ULL,
        365310924ULL, 1966671750ULL, 3292685171ULL, 331099293ULL,
        1573581988ULL, 3730001075ULL, 2114886462ULL, 429775716ULL,
        57094938ULL, 1321759700ULL, 1544733806ULL, 4212594609ULL,
        2175410558ULL, 3374240236ULL, 4124708467ULL, 4000379620ULL,
        2909430069ULL, 960018985ULL, 3601707582ULL, 443053259ULL,
        1218353539ULL, 1670632391ULL, 1727392122ULL, 2784951487ULL,
        2481713080ULL, 766670702ULL, 1748411460ULL, 3896941215ULL,
        4074805399ULL, 3436125804ULL, 2451266564ULL, 875001912ULL,
        3432880105ULL, 2205685918ULL, 2229724174ULL, 1584253406ULL,
        2393580504ULL, 3187306808ULL, 3961684887ULL, 1658081136ULL,
        2169997687ULL, 3427929537ULL, 1230009001ULL, 787750860ULL,
        1510254211ULL, 781084165ULL, 2974020319ULL, 44ULL};
    rng_validation<vsmc::Threefry4x32>(
        Threefry4x32Result, "Threefry4x32");

    std::vector<unsigned long long> Threefry2x64Result = {
        6587418233440704010ULL, 922607945289313709ULL,
        14863425584661357569ULL, 8366499438842246568ULL,
        7459583178209588857ULL, 4341049198327997016ULL,
        2142343144172995027ULL, 7523051694790286047ULL,
        5160574532984340816ULL, 9194830205800408763ULL,
        3131098958709831207ULL, 17400813698458991012ULL,
        18329199285825886116ULL, 7619210131774476516ULL,
        14550090069204230542ULL, 9969411945551358270ULL,
        2088246116591219002ULL, 10298303720705153390ULL,
        1903346978165959307ULL, 16737453259476682233ULL,
        16471144238978361467ULL, 16061773712243385075ULL,
        17140024010031247152ULL, 15336641831225437302ULL,
        7851590713267384422ULL, 6168972582221363678ULL,
        16837777807755455428ULL, 16803019700919584460ULL,
        129016649478878310ULL, 10385716191783771989ULL,
        16974842600788632809ULL, 14474764081319200051ULL,
        5452041686175608601ULL, 14966519046825396431ULL,
        15006085919695417091ULL, 18156040825660799327ULL,
        10974229701985622149ULL, 10158317891628574699ULL,
        682714384032315478ULL, 10094451520337859365ULL, 514923734528049335ULL,
        9413022251204842157ULL, 12768830906701707127ULL, 78209100079398715ULL,
        590738149020479043ULL, 4834952516581644761ULL, 5867751209029899549ULL,
        2288214489420888668ULL, 15528562910702878611ULL,
        14075097022818740397ULL, 13083816830636422398ULL,
        11643126780691293350ULL, 4462150952979220291ULL,
        10350653676414502046ULL, 8382065844199425538ULL,
        16733286051063922736ULL, 5867293444299497537ULL,
        14563469547120155875ULL, 13851905626533263880ULL,
        12298902792128120390ULL, 4608441571922363330ULL,
        16872753955680508327ULL, 16545103150795410077ULL,
        16934527485928077500ULL, 11305718803224170638ULL,
        8190660921315919988ULL, 11478838648140357726ULL,
        3208131281483851429ULL, 9905475976280520386ULL,
        1836223428090744828ULL, 10702219711774523881ULL,
        5612839703212048661ULL, 8388567972950377117ULL,
        12330762137301405599ULL, 7147420788695825663ULL,
        17162388781270739570ULL, 14765238474517261540ULL,
        7605549319423042193ULL, 3116721681098513264ULL,
        8740545241800862790ULL, 11846330799740316891ULL,
        1693299367495359784ULL, 14932711833343191466ULL,
        13890030381488082337ULL, 2665259664194682296ULL,
        12925005556405688263ULL, 12437588553278275097ULL,
        7016186509563084253ULL, 14122265029557927930ULL,
        996409059862699526ULL, 12302765996885207331ULL,
        14416185114848647403ULL, 15180400626601542658ULL,
        16096300297349912074ULL, 6343226359364436775ULL,
        13301976048850739582ULL, 15032256000915451172ULL,
        16648524814763982427ULL, 491535854765621175ULL,
        8179707882298974426ULL, 95ULL};
    rng_validation<vsmc::Threefry2x64>(
        Threefry2x64Result, "Threefry2x64");

    std::vector<unsigned long long> Threefry4x64Result = {
        3925897152597938002ULL, 1593725430291856998ULL,
        4153848473662934726ULL, 17515109607213902335ULL,
        17111640339533506295ULL, 4934353305630693528ULL,
        17811454045609800464ULL, 16022337223824491849ULL,
        4212233813287760189ULL, 5248728824309090599ULL,
        6382762843102419571ULL, 8784629853176381034ULL,
        8527366336945210006ULL, 17804561367668509428ULL,
        17041828765430495700ULL, 5990502758443390465ULL,
        5235778047071056699ULL, 5830826922563198427ULL,
        14997371527587218762ULL, 13997843459073252230ULL,
        13063810989912874542ULL, 8603112947521667203ULL,
        8341142921262310927ULL, 924511780866776792ULL, 7979453151818931459ULL,
        168301575075086134ULL, 8625297230597261972ULL, 911996110852711445ULL,
        16717412388901062117ULL, 15181561854312677889ULL,
        13775298372238406397ULL, 8473644221861591119ULL,
        5403939826310550603ULL, 3212923184682265358ULL,
        5691588554661620657ULL, 10136534467600082275ULL,
        12515694207338603849ULL, 3251788443016770948ULL,
        5184671860650632041ULL, 9540434821133576801ULL, 850507386746134169ULL,
        16292533358281323778ULL, 11247715358563236817ULL,
        4356868601412157191ULL, 3975128502197482008ULL, 909071751227214472ULL,
        17845033314386016770ULL, 4408095413790440303ULL,
        3744853938374541029ULL, 4801474438060953027ULL,
        8382440762194322060ULL, 1762367883350342565ULL,
        9424422161094083766ULL, 16447825091256776204ULL,
        1697252268502785517ULL, 4496091816999321127ULL,
        2019722580460156865ULL, 13020844749295782885ULL, 85171173952178998ULL,
        3371166850056185620ULL, 14334924082596746673ULL,
        12018103873229442539ULL, 5333015197434363005ULL,
        3711223215095205769ULL, 10654914237161755602ULL,
        330074762182901827ULL, 2163062940654811552ULL, 3691907874260661086ULL,
        9592568823065862468ULL, 12504076516956749592ULL,
        17845769106694981076ULL, 1992105841965730968ULL,
        5696727913762571660ULL, 13076474436660884058ULL,
        12333007197816108954ULL, 9991063431487047807ULL,
        9701392882023480250ULL, 8634826361338755347ULL,
        12713360119699553583ULL, 14387097466270528203ULL,
        1950499581192909562ULL, 3427757613452781042ULL,
        7044488827619633797ULL, 18349783839536030510ULL,
        18272017861525105413ULL, 9113524046075944496ULL,
        13899113120156631536ULL, 1259682791194237926ULL,
        4513182957490050012ULL, 2232996808000998525ULL,
        5466438615289456302ULL, 310785573616604153ULL, 1540899326153821420ULL,
        11323946356711887507ULL, 10818938406447144617ULL,
        16774051211549599565ULL, 9093562834670167937ULL,
        4785193166798545310ULL, 2409366490122510774ULL,
        2564164278159198574ULL, 5ULL};
    rng_validation<vsmc::Threefry4x64>(
        Threefry4x64Result, "Threefry4x64");

    return 0;
}