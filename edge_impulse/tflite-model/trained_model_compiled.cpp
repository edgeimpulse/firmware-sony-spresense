/* Generated by Edge Impulse
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/
// Generated on: 18.06.2021 19:43:30

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "edge-impulse-sdk/tensorflow/lite/c/builtin_op_data.h"
#include "edge-impulse-sdk/tensorflow/lite/c/common.h"
#include "edge-impulse-sdk/tensorflow/lite/micro/kernels/micro_ops.h"

#if EI_CLASSIFIER_PRINT_STATE
#if defined(__cplusplus) && EI_C_LINKAGE == 1
extern "C" {
    extern void ei_printf(const char *format, ...);
}
#else
extern void ei_printf(const char *format, ...);
#endif
#endif

#if defined __GNUC__
#define ALIGN(X) __attribute__((aligned(X)))
#elif defined _MSC_VER
#define ALIGN(X) __declspec(align(X))
#elif defined __TASKING__
#define ALIGN(X) __align(X)
#endif

namespace {

constexpr int kTensorArenaSize = 288;

#if defined(EI_CLASSIFIER_ALLOCATION_STATIC)
uint8_t tensor_arena[kTensorArenaSize] ALIGN(16);
#elif defined(EI_CLASSIFIER_ALLOCATION_STATIC_HIMAX)
#pragma Bss(".tensor_arena")
uint8_t tensor_arena[kTensorArenaSize] ALIGN(16);
#pragma Bss()
#elif defined(EI_CLASSIFIER_ALLOCATION_STATIC_HIMAX_GNU)
uint8_t tensor_arena[kTensorArenaSize] ALIGN(16) __attribute__((section(".tensor_arena")));
#else
#define EI_CLASSIFIER_ALLOCATION_HEAP 1
uint8_t* tensor_arena = NULL;
#endif

static uint8_t* tensor_boundary;
static uint8_t* current_location;

template <int SZ, class T> struct TfArray {
  int sz; T elem[SZ];
};
enum used_operators_e {
  OP_FULLY_CONNECTED, OP_SOFTMAX,  OP_LAST
};
struct TensorInfo_t { // subset of TfLiteTensor used for initialization from constant memory
  TfLiteAllocationType allocation_type;
  void* data;
  TfLiteIntArray* dims;
  size_t bytes;
};
struct NodeInfo_t { // subset of TfLiteNode used for initialization from constant memory
  struct TfLiteIntArray* inputs;
  struct TfLiteIntArray* outputs;
  void* builtin_data;
  used_operators_e used_op_index;
};

TfLiteContext ctx{};
TfLiteTensor tflTensors[11];
TfLiteRegistration registrations[OP_LAST];
TfLiteNode tflNodes[4];

const TfArray<2, int> tensor_dimension0 = { 2, { 1,33 } };
const ALIGN(8) float tensor_data1[20] = { -0.00075289892265573144, 0.25658360123634338, -0.23587076365947723, 0.011300439015030861, 0.54371809959411621, 0.31057509779930115, -0.015062404796481133, 0.33908054232597351, 0.46735432744026184, -0.0087245041504502296, 0.76048034429550171, -0.17259438335895538, 0.112441286444664, 0.014623193070292473, 0.82903075218200684, 0.51425212621688843, -0.011085406877100468, 0.84291130304336548, 0.096076026558876038, -0.0073170214891433716, };
const TfArray<1, int> tensor_dimension1 = { 1, { 20 } };
const ALIGN(8) float tensor_data2[10] = { 0.019532050937414169, -0.011763913556933403, 0.50659793615341187, 0.77090167999267578, 0.12201302498579025, -0.015334728173911572, 0.14537382125854492, -0.14080679416656494, 0.61982607841491699, -0.1048121377825737, };
const TfArray<1, int> tensor_dimension2 = { 1, { 10 } };
const ALIGN(8) float tensor_data3[4] = { 0.45249274373054504, -0.20760074257850647, 0.24450193345546722, -0.42199599742889404, };
const TfArray<1, int> tensor_dimension3 = { 1, { 4 } };
const ALIGN(8) float tensor_data4[20*33] = { 
  1.1587105989456177, -0.089692950248718262, 0.17145606875419617, 0.053736697882413864, 0.17180073261260986, 0.094172053039073944, -0.21512509882450104, -0.24265500903129578, 1.0479773283004761, 0.67617183923721313, 0.87146329879760742, 1.1006369590759277, -0.23503933846950531, 0.64589023590087891, 0.11473348736763, -0.023591244593262672, 0.22556126117706299, -0.23007966578006744, 0.22656747698783875, 1.1729198694229126, 0.40526798367500305, -0.13972841203212738, 1.8097666501998901, -0.3667323887348175, 0.86357665061950684, -0.030965562909841537, -0.32309669256210327, 0.22688235342502594, -0.046528585255146027, -0.11683471500873566, 1.4214103221893311, 0.77767729759216309, 1.0968829393386841, 
  1.2496422529220581, 0.05681416392326355, 0.210251584649086, 0.021223921328783035, 0.37339994311332703, 0.34386703372001648, 0.24612782895565033, 0.27284219861030579, 0.87379002571105957, 1.3809276819229126, 1.1632446050643921, 0.86912071704864502, 0.044317778199911118, -0.56006646156311035, -0.0045752301812171936, -0.13093589246273041, -0.26965796947479248, -0.41521108150482178, 0.23038122057914734, 0.81787002086639404, 0.92618638277053833, 0.65350311994552612, 0.26772052049636841, -0.52440249919891357, 0.14441987872123718, -0.17460009455680847, 0.097643248736858368, -0.0097805270925164223, -0.36575502157211304, 0.018285304307937622, 0.83120429515838623, 1.1312631368637085, 0.11843202263116837, 
  0.24705581367015839, -0.28730446100234985, -0.053260505199432373, 0.3854580819606781, 0.13469217717647552, 0.12793445587158203, -0.25951477885246277, -0.07890358567237854, 0.2524179220199585, 0.36116853356361389, -0.081988677382469177, -0.14784659445285797, -0.18079875409603119, -0.14245277643203735, 0.078474827110767365, -0.21931883692741394, -0.12873250246047974, 0.013763941824436188, 0.27623525261878967, -0.34996739029884338, 0.55623531341552734, -0.023370031267404556, -0.44996070861816406, 0.039856720715761185, -0.46019229292869568, 0.24758091568946838, -0.22145088016986847, -0.31092321872711182, -0.22448568046092987, -0.19977740943431854, -0.37091025710105896, 0.22127258777618408, -0.44642549753189087, 
  1.1830135583877563, -0.21769312024116516, -0.40063080191612244, 0.32009652256965637, 0.22136868536472321, 0.07537485659122467, -0.24691992998123169, 0.051169693470001221, 0.49511516094207764, 1.1295063495635986, 0.89723485708236694, 1.4289251565933228, 0.31795287132263184, 0.65049690008163452, -0.028243878856301308, -0.37150359153747559, -0.13498418033123016, -0.17426121234893799, 0.31376758217811584, 1.3069833517074585, 0.97844541072845459, 0.99277132749557495, 1.2086358070373535, 0.20741759240627289, -0.02122923918068409, 0.17766857147216797, -0.3143610954284668, -0.19456599652767181, 0.14755594730377197, 0.061264455318450928, 0.40958100557327271, 1.2279112339019775, 0.5036466121673584, 
  -0.6965143084526062, 0.36389756202697754, -0.24298055469989777, 0.21383354067802429, -0.10101157426834106, 0.23246534168720245, -0.31826213002204895, -0.12980036437511444, -0.76521188020706177, -0.21551144123077393, -0.12418242543935776, -0.49019372463226318, 0.054378796368837357, 0.24667796492576599, -0.070736818015575409, 0.41598427295684814, -0.24617111682891846, -0.31133943796157837, 0.075863122940063477, -1.0189530849456787, 0.55707037448883057, 0.11335311830043793, -0.90934234857559204, 0.19736334681510925, -0.52354192733764648, -0.098970793187618256, 0.4836139976978302, 0.10706617683172226, 0.29349762201309204, -0.003009408712387085, -0.98839795589447021, -0.84206867218017578, -0.73084133863449097, 
  1.2420667409896851, 0.096995852887630463, 0.30619430541992188, 0.089530587196350098, 0.23729944229125977, 0.22430421411991119, 0.18398286402225494, 0.11178964376449585, 0.71328121423721313, 0.73087245225906372, 0.71962440013885498, 1.0556079149246216, 0.65575242042541504, 0.11194268614053726, 0.35022920370101929, 0.058735813945531845, -0.20652006566524506, 0.0071393605321645737, 0.22444388270378113, 1.1225235462188721, 1.0876075029373169, 0.69608741998672485, 0.5881350040435791, 0.27083420753479004, -0.2748127281665802, 0.11448373645544052, -0.25954931974411011, -0.034156147390604019, 0.19485913217067719, 0.10951632261276245, 0.53939151763916016, 1.2926673889160156, 0.31382578611373901, 
  -0.47900700569152832, 0.59259635210037231, -0.51397132873535156, -0.13933664560317993, -0.22951588034629822, 0.059143126010894775, 0.058411069214344025, -0.07543867826461792, -0.81769335269927979, 0.57786965370178223, -0.041004214435815811, -0.33854493498802185, -0.087423704564571381, -0.31567871570587158, 0.27216547727584839, 0.32290208339691162, 0.056291479617357254, 0.19920676946640015, 0.089851409196853638, -0.58186626434326172, 0.78496026992797852, 0.37323787808418274, -0.19715045392513275, -0.13805121183395386, -0.43917089700698853, 0.0039123250171542168, -0.27891793847084045, -0.051044952124357224, -0.04530315101146698, -0.091368108987808228, -0.47827187180519104, -0.42489218711853027, -0.50123000144958496, 
  -1.0080883502960205, 0.59696221351623535, -0.24531891942024231, 0.028863510116934776, -0.15344542264938354, 0.28143027424812317, 0.017602862790226936, -0.30905058979988098, -0.3652341365814209, -1.2224670648574829, -0.5056910514831543, -0.79703503847122192, -0.10152250528335571, 0.18920263648033142, -0.047908701002597809, 0.28128686547279358, -0.048459317535161972, -0.37856689095497131, 0.019424885511398315, -0.36539867520332336, -1.2663323879241943, -1.078853964805603, -0.067920461297035217, 0.1306561678647995, 0.5338025689125061, -0.22156019508838654, 0.4950699508190155, 0.13329432904720306, 0.14814206957817078, 0.23151704668998718, -0.16826610267162323, -0.29064172506332397, -0.29449230432510376, 
  1.2910501956939697, 0.2539266049861908, 0.4119681715965271, -0.1622534841299057, 0.44741392135620117, 0.21937656402587891, -0.048032891005277634, -0.066160380840301514, 0.98531824350357056, 0.81164592504501343, 0.66489136219024658, 1.4316918849945068, 0.30378249287605286, -0.2390017956495285, -0.23276501893997192, -0.19930166006088257, -0.028879754245281219, -0.19920799136161804, 0.23968318104743958, 0.49223479628562927, 1.0081679821014404, 0.82576531171798706, 0.23459507524967194, -0.24936942756175995, -0.38133201003074646, 0.182932049036026, -0.6381344199180603, 0.067067235708236694, 0.20073053240776062, -0.0051628351211547852, 0.084347628057003021, 0.69184821844100952, -0.16037102043628693, 
  1.0757232904434204, 0.22879323363304138, 0.51891160011291504, 0.016448656097054482, -0.21213482320308685, 0.046403955668210983, 0.032562196254730225, -0.12624906003475189, 1.2091614007949829, 0.51481884717941284, 0.48256775736808777, 0.63524717092514038, -0.354108065366745, 0.039170481264591217, -0.010301684029400349, -0.38945555686950684, -0.020175101235508919, 0.26809215545654297, 0.27086570858955383, 0.82041764259338379, -0.341229647397995, -0.50982421636581421, 1.5207729339599609, 0.3005555272102356, 1.6650046110153198, 0.0058658472262322903, 0.093796223402023315, -0.015711214393377304, 0.48918253183364868, 0.18952301144599915, 1.7354519367218018, 0.72186416387557983, 1.0307861566543579, 
  -0.90594792366027832, -0.39260447025299072, -0.67167896032333374, 0.10001154989004135, 0.6370246410369873, -0.11255790293216705, 0.10708120465278625, -0.010768383741378784, -0.96738737821578979, -0.50876718759536743, 0.085596829652786255, -0.34551453590393066, 0.18203698098659515, 0.27736327052116394, 0.10518568754196167, 0.18993999063968658, -0.024209002032876015, 0.38995623588562012, -0.10325177013874054, -1.1236928701400757, -0.15160901844501495, 0.15835736691951752, -0.25137171149253845, -0.1393938809633255, -0.10305240005254745, 0.16376368701457977, 0.16735844314098358, 0.15408653020858765, 0.1367611289024353, -0.27562448382377625, -0.32932278513908386, -0.5451667308807373, -0.33538702130317688, 
  0.09111902117729187, 0.025864111259579659, -0.11587293446063995, -0.08993104100227356, 0.0056612999178469181, 0.28620865941047668, 0.046084299683570862, 0.23238977789878845, 0.14396929740905762, 0.15353870391845703, -0.22574643790721893, 0.16512040793895721, 0.0078041166998445988, -0.24859237670898438, -0.18389001488685608, 0.41109523177146912, 0.23479248583316803, 0.4008941650390625, 0.25606814026832581, 0.12440066039562225, 0.26494082808494568, -0.040001079440116882, -0.20470359921455383, -0.0031525494996458292, 0.10060038417577744, -0.20646482706069946, 0.047735515981912613, 0.35144838690757751, -0.097947627305984497, 0.20253083109855652, -0.48894718289375305, -0.018410675227642059, -0.35793972015380859, 
  -0.44012179970741272, 0.27510666847229004, -0.12402656674385071, -0.32462719082832336, 0.12494503706693649, -0.10693053901195526, 0.088716953992843628, -0.14490216970443726, -0.54555928707122803, -0.010962286032736301, -0.25830796360969543, -0.53567564487457275, -0.10789180546998978, 0.25244778394699097, 0.26230683922767639, -0.10506562888622284, 0.17675529420375824, -0.051232043653726578, -0.31777080893516541, -0.74380958080291748, -0.073484286665916443, 0.15778772532939911, -0.093812540173530579, 0.044700309634208679, -0.33148068189620972, -0.033569801598787308, -0.16490218043327332, 0.076817922294139862, 0.3111245334148407, -0.13972514867782593, -0.41836315393447876, -0.53269767761230469, -0.4299619197845459, 
  0.44191178679466248, -0.078151710331439972, -0.0062222923152148724, 0.17463906109333038, -0.041912570595741272, -0.10559637844562531, 0.13627852499485016, -0.065433204174041748, 0.13592956960201263, -0.088787682354450226, 0.30937784910202026, 0.6579362154006958, 0.30204972624778748, -0.007954978384077549, -0.18026189506053925, -0.14989300072193146, 0.038080193102359772, 0.11192335933446884, 0.090664267539978027, 0.31559213995933533, -0.98719918727874756, -0.33751338720321655, 0.55321997404098511, -0.4270814061164856, -0.12941084802150726, 0.3690095841884613, 0.57269203662872314, 0.057841207832098007, 0.35700756311416626, -0.14747343957424164, -0.39233982563018799, -0.12818035483360291, 0.35969698429107666, 
  -1.1113876104354858, -0.010453230701386929, 0.20398424565792084, 0.013444161973893642, 0.29632213711738586, -0.18035081028938293, 0.15161392092704773, 0.20572462677955627, -0.99017792940139771, -1.2332887649536133, -0.90085053443908691, -0.64041584730148315, -0.035364590585231781, 0.10118702054023743, 0.021144818514585495, 0.07991732656955719, -0.03414427861571312, -0.025190779939293861, -0.14156842231750488, -1.0885689258575439, -1.0173079967498779, -0.41838362812995911, -0.040911681950092316, 0.11060900241136551, 0.65019518136978149, 0.14227734506130219, 0.26271656155586243, -0.044926326721906662, -0.077689014375209808, 0.020872116088867188, -0.034566052258014679, -0.64409607648849487, 0.33537337183952332, 
  -0.31279361248016357, 0.47743329405784607, 0.14985983073711395, 0.20780396461486816, 0.04240909218788147, 0.24373447895050049, -0.24497395753860474, -0.1645902693271637, -0.52956295013427734, -0.2978324294090271, -0.20721110701560974, 0.14933417737483978, 0.067558251321315765, 0.16037070751190186, 0.38749119639396667, 0.022405745461583138, 0.01605437695980072, 0.36290550231933594, 0.20569190382957458, -0.34754443168640137, -0.49334284663200378, -0.21206904947757721, -0.0046703615225851536, -0.22538669407367706, -0.25567129254341125, -0.089594677090644836, 0.48502954840660095, 0.025312971323728561, -0.010602064430713654, 0.030603110790252686, -1.0995087623596191, -1.0481011867523193, -0.45376864075660706, 
  0.43564659357070923, 0.33445939421653748, 0.33133092522621155, 0.095838591456413269, 0.14455202221870422, 0.066138960421085358, 0.031958881765604019, -0.15881600975990295, 0.54299181699752808, 0.7463565468788147, 0.463034987449646, 0.55234181880950928, -0.14217719435691833, -0.04418395459651947, -0.12138906121253967, 0.045131750404834747, 0.11056815087795258, 0.030621180310845375, -0.27817505598068237, 0.91180539131164551, 0.71245235204696655, 0.24759246408939362, 0.2857077419757843, 0.54366540908813477, 0.31950840353965759, 0.07296464592218399, -0.40858343243598938, 0.082798995077610016, 0.3108813464641571, 0.3098846971988678, 0.51813715696334839, 0.81775569915771484, 0.1716398149728775, 
  -0.76636683940887451, -0.083969198167324066, -0.94952118396759033, -0.28801658749580383, 0.29121679067611694, -0.13969939947128296, 0.055467337369918823, 0.23674061894416809, -1.0615913867950439, -0.24533770978450775, 0.0029414468444883823, -0.23869192600250244, 0.025253500789403915, -0.62014478445053101, -0.020644305273890495, 0.34345176815986633, 0.12371620535850525, 0.039819817990064621, -0.17544406652450562, -0.91257560253143311, 0.15650080144405365, 0.20134972035884857, -0.59043163061141968, -0.64950466156005859, -0.2094733864068985, -0.24262000620365143, -0.44572290778160095, 0.15527310967445374, 0.015655398368835449, 0.17338290810585022, -0.61750161647796631, -0.62832623720169067, -0.70077860355377197, 
  0.8586432933807373, 0.020787779241800308, 0.33599978685379028, 0.15736506879329681, 0.19638483226299286, 0.081185810267925262, -0.05529731884598732, -0.22608831524848938, 0.0017935796640813351, 0.053713049739599228, 0.19953787326812744, 1.1040871143341064, 0.33151742815971375, 0.65790873765945435, -0.27264273166656494, 0.055080290883779526, -0.12123900651931763, -0.079448215663433075, -0.27671027183532715, 1.0045892000198364, 0.45744654536247253, 0.32094278931617737, 0.88673645257949829, 0.74902886152267456, 0.42240279912948608, -0.10847704857587814, 0.2820107638835907, 0.047411799430847168, 0.14092513918876648, -0.020534813404083252, 0.92922729253768921, 1.1017439365386963, 1.1269159317016602, 
  0.8544735312461853, 0.30984982848167419, 0.59681034088134766, 0.27026188373565674, 0.34517911076545715, 0.082969039678573608, 0.098159536719322205, -0.2854498028755188, 1.1529349088668823, 1.1045479774475098, 0.50885313749313354, 1.2109390497207642, 0.088768988847732544, 0.085894189774990082, 0.39695635437965393, -0.047994229942560196, 0.018307484686374664, -0.37396222352981567, 0.094955027103424072, 1.1276369094848633, 0.26965856552124023, 0.54348433017730713, 0.87047827243804932, -0.26043751835823059, 0.25500732660293579, 0.25791999697685242, -0.32752123475074768, 0.35607284307479858, -0.29776462912559509, -0.32846915721893311, 0.25994423031806946, 0.59976667165756226, 0.69808918237686157, 
};
const TfArray<2, int> tensor_dimension4 = { 2, { 20,33 } };
const ALIGN(8) float tensor_data5[10*20] = { 
  -0.31950533390045166, -0.0015343516133725643, 0.23893040418624878, 0.16496095061302185, 0.47547781467437744, -0.15304489433765411, -0.065639160573482513, 0.30808281898498535, -0.38580542802810669, -0.36397501826286316, -0.26211127638816833, -0.29004031419754028, 0.38931295275688171, 0.39174565672874451, -0.026553735136985779, 0.099812015891075134, -0.34919488430023193, -0.039794709533452988, -0.093059971928596497, 0.10473828017711639, 
  0.41199177503585815, 0.60070353746414185, 0.16690428555011749, 0.51703327894210815, 0.071988970041275024, 0.371285080909729, 0.47004446387290955, -0.99348795413970947, 0.28189849853515625, -0.017911586910486221, 0.086318999528884888, -0.14721165597438812, -0.037803784012794495, -0.035153824836015701, -0.58563679456710815, -0.4384198784828186, 0.14332640171051025, 0.58088153600692749, 0.53883558511734009, -0.086515195667743683, 
  -0.75940263271331787, 0.53938573598861694, -0.012759987264871597, -0.50113475322723389, 0.73958796262741089, 0.08856925368309021, 0.38988003134727478, 0.017797406762838364, 0.67936772108078003, -0.046076539903879166, 0.15588417649269104, 0.038014091551303864, 0.24171817302703857, -0.61735224723815918, 0.45038384199142456, -0.088121585547924042, -0.17273752391338348, 1.3366910219192505, -0.67340576648712158, -0.1807730495929718, 
  -0.6112133264541626, -0.50443446636199951, -0.37848031520843506, -0.37363484501838684, 0.66164129972457886, 0.24836175143718719, 0.27856865525245667, 0.67251521348953247, 0.28078100085258484, -0.63073831796646118, 1.1409662961959839, 0.23905524611473083, 0.010193623602390289, -0.38813498616218567, 0.99125415086746216, 0.59224450588226318, 0.0039535313844680786, 0.87370759248733521, 0.17169874906539917, -0.51211512088775635, 
  0.71045011281967163, 1.1238372325897217, -0.39836525917053223, 0.083547666668891907, -0.95617181062698364, 0.78255641460418701, -0.30069255828857422, -0.033613789826631546, 0.49801582098007202, 1.2972478866577148, 0.13114190101623535, -0.36358040571212769, -0.48550888895988464, 0.70975750684738159, 0.24751809239387512, 0.068754047155380249, 0.42926681041717529, -0.90740865468978882, 0.80756205320358276, 0.63911473751068115, 
  1.3846255540847778, 1.3346874713897705, -0.4315619170665741, 0.75867593288421631, -0.99559706449508667, 0.57787817716598511, -0.069297000765800476, -0.93143028020858765, 1.4021518230438232, 0.48930680751800537, -0.29959684610366821, 0.29537701606750488, -0.42846143245697021, 0.99791616201400757, -0.36336809396743774, 0.44550624489784241, -0.071026444435119629, -0.032255019992589951, 0.042454518377780914, 1.1081607341766357, 
  0.74187982082366943, -0.14645949006080627, -0.15686811506748199, 0.55388307571411133, 0.132075235247612, -0.148488849401474, -0.11482203006744385, 0.33237418532371521, 0.27543270587921143, 0.73760885000228882, -0.13787248730659485, 0.23008421063423157, -0.65898787975311279, 0.13807220757007599, 0.32771223783493042, 0.03907785564661026, 0.30479201674461365, -0.63702327013015747, 0.37833639979362488, -0.16735973954200745, 
  -0.072760134935379028, 0.23367796838283539, -0.22337539494037628, -0.40529415011405945, -0.037111438810825348, -0.25902312994003296, -0.15727639198303223, -0.29965773224830627, 0.19504857063293457, 0.15223206579685211, 0.31102493405342102, 0.36794430017471313, -0.10213513672351837, -0.020616095513105392, -0.13053713738918304, -0.22402812540531158, -0.30713313817977905, -0.049926478415727615, -0.23294436931610107, 0.092174045741558075, 
  -0.55274111032485962, -0.12203346192836761, 0.089954957365989685, 0.26738330721855164, 0.86541968584060669, 0.094913788139820099, 0.16468602418899536, -0.14997361600399017, -0.0005750044365413487, -0.53161042928695679, 0.76132375001907349, 0.38997891545295715, 0.32465112209320068, 0.36488494277000427, 0.66357797384262085, 0.31500175595283508, -0.35630986094474792, 0.98355907201766968, 0.052115168422460556, -0.09880559891462326, 
  0.014194963499903679, 0.95929455757141113, 0.12371824681758881, 0.35891193151473999, -0.32888761162757874, 0.61789238452911377, 0.48489513993263245, -0.67898321151733398, 0.58612275123596191, 0.15862147510051727, -0.27949813008308411, 0.24437318742275238, -0.22956548631191254, -0.20150932669639587, -0.85567295551300049, 0.15577450394630432, 0.7066032886505127, 0.72341108322143555, 0.21581733226776123, 0.66700887680053711, 
};
const TfArray<2, int> tensor_dimension5 = { 2, { 10,20 } };
const ALIGN(8) float tensor_data6[4*10] = { 
  -0.21225674450397491, -2.2252168655395508, 0.95733678340911865, 1.229886531829834, -1.5081713199615479, -2.1574242115020752, -1.6809475421905518, -0.020526885986328125, 0.80089706182479858, -1.2403197288513184, 
  0.57946574687957764, 0.64446830749511719, 0.47916257381439209, 0.59744763374328613, -0.25165772438049316, -0.0051823733374476433, 0.13109864294528961, -0.39976373314857483, 0.49950700998306274, 0.51683849096298218, 
  -0.92380315065383911, -0.89461159706115723, -1.7157931327819824, 0.20973899960517883, 0.59508079290390015, 0.55116379261016846, 0.62403833866119385, -0.070111393928527832, 0.17181350290775299, -0.61136502027511597, 
  -0.67170542478561401, 0.5365874171257019, 0.20717865228652954, -0.9520564079284668, 0.35116523504257202, 0.38973379135131836, -0.084877341985702515, -0.48895749449729919, -1.1713943481445312, 1.0917503833770752, 
};
const TfArray<2, int> tensor_dimension6 = { 2, { 4,10 } };
const TfArray<2, int> tensor_dimension7 = { 2, { 1,20 } };
const TfArray<2, int> tensor_dimension8 = { 2, { 1,10 } };
const TfArray<2, int> tensor_dimension9 = { 2, { 1,4 } };
const TfArray<2, int> tensor_dimension10 = { 2, { 1,4 } };
const TfLiteFullyConnectedParams opdata0 = { kTfLiteActRelu, kTfLiteFullyConnectedWeightsFormatDefault, false, false };
const TfArray<3, int> inputs0 = { 3, { 0,4,1 } };
const TfArray<1, int> outputs0 = { 1, { 7 } };
const TfLiteFullyConnectedParams opdata1 = { kTfLiteActRelu, kTfLiteFullyConnectedWeightsFormatDefault, false, false };
const TfArray<3, int> inputs1 = { 3, { 7,5,2 } };
const TfArray<1, int> outputs1 = { 1, { 8 } };
const TfLiteFullyConnectedParams opdata2 = { kTfLiteActNone, kTfLiteFullyConnectedWeightsFormatDefault, false, false };
const TfArray<3, int> inputs2 = { 3, { 8,6,3 } };
const TfArray<1, int> outputs2 = { 1, { 9 } };
const TfLiteSoftmaxParams opdata3 = { 1 };
const TfArray<1, int> inputs3 = { 1, { 9 } };
const TfArray<1, int> outputs3 = { 1, { 10 } };
const TensorInfo_t tensorData[] = {
  { kTfLiteArenaRw, tensor_arena + 0, (TfLiteIntArray*)&tensor_dimension0, 132, },
  { kTfLiteMmapRo, (void*)tensor_data1, (TfLiteIntArray*)&tensor_dimension1, 80, },
  { kTfLiteMmapRo, (void*)tensor_data2, (TfLiteIntArray*)&tensor_dimension2, 40, },
  { kTfLiteMmapRo, (void*)tensor_data3, (TfLiteIntArray*)&tensor_dimension3, 16, },
  { kTfLiteMmapRo, (void*)tensor_data4, (TfLiteIntArray*)&tensor_dimension4, 2640, },
  { kTfLiteMmapRo, (void*)tensor_data5, (TfLiteIntArray*)&tensor_dimension5, 800, },
  { kTfLiteMmapRo, (void*)tensor_data6, (TfLiteIntArray*)&tensor_dimension6, 160, },
  { kTfLiteArenaRw, tensor_arena + 144, (TfLiteIntArray*)&tensor_dimension7, 80, },
  { kTfLiteArenaRw, tensor_arena + 0, (TfLiteIntArray*)&tensor_dimension8, 40, },
  { kTfLiteArenaRw, tensor_arena + 48, (TfLiteIntArray*)&tensor_dimension9, 16, },
  { kTfLiteArenaRw, tensor_arena + 0, (TfLiteIntArray*)&tensor_dimension10, 16, },
};const NodeInfo_t nodeData[] = {
  { (TfLiteIntArray*)&inputs0, (TfLiteIntArray*)&outputs0, const_cast<void*>(static_cast<const void*>(&opdata0)), OP_FULLY_CONNECTED, },
  { (TfLiteIntArray*)&inputs1, (TfLiteIntArray*)&outputs1, const_cast<void*>(static_cast<const void*>(&opdata1)), OP_FULLY_CONNECTED, },
  { (TfLiteIntArray*)&inputs2, (TfLiteIntArray*)&outputs2, const_cast<void*>(static_cast<const void*>(&opdata2)), OP_FULLY_CONNECTED, },
  { (TfLiteIntArray*)&inputs3, (TfLiteIntArray*)&outputs3, const_cast<void*>(static_cast<const void*>(&opdata3)), OP_SOFTMAX, },
};
static std::vector<void*> overflow_buffers;
static TfLiteStatus AllocatePersistentBuffer(struct TfLiteContext* ctx,
                                                 size_t bytes, void** ptr) {
  if (current_location - bytes < tensor_boundary) {
    // OK, this will look super weird, but.... we have CMSIS-NN buffers which
    // we cannot calculate beforehand easily.
    *ptr = malloc(bytes);
    if (*ptr == NULL) {
      printf("ERR: Failed to allocate persistent buffer of size %d\n", (int)bytes);
      return kTfLiteError;
    }
    overflow_buffers.push_back(*ptr);
    return kTfLiteOk;
  }

  current_location -= bytes;

  *ptr = current_location;
  return kTfLiteOk;
}
typedef struct {
  size_t bytes;
  void *ptr;
} scratch_buffer_t;
static std::vector<scratch_buffer_t> scratch_buffers;

static TfLiteStatus RequestScratchBufferInArena(struct TfLiteContext* ctx, size_t bytes,
                                                int* buffer_idx) {
  scratch_buffer_t b;
  b.bytes = bytes;

  TfLiteStatus s = AllocatePersistentBuffer(ctx, b.bytes, &b.ptr);
  if (s != kTfLiteOk) {
    return s;
  }

  scratch_buffers.push_back(b);

  *buffer_idx = scratch_buffers.size() - 1;

  return kTfLiteOk;
}

static void* GetScratchBuffer(struct TfLiteContext* ctx, int buffer_idx) {
  if (buffer_idx > static_cast<int>(scratch_buffers.size()) - 1) {
    return NULL;
  }
  return scratch_buffers[buffer_idx].ptr;
}
} // namespace

  TfLiteStatus trained_model_init( void*(*alloc_fnc)(size_t,size_t) ) {
#ifdef EI_CLASSIFIER_ALLOCATION_HEAP
  tensor_arena = (uint8_t*) alloc_fnc(16, kTensorArenaSize);
  if (!tensor_arena) {
    printf("ERR: failed to allocate tensor arena\n");
    return kTfLiteError;
  }
#endif
  tensor_boundary = tensor_arena;
  current_location = tensor_arena + kTensorArenaSize;
  ctx.AllocatePersistentBuffer = &AllocatePersistentBuffer;
  ctx.RequestScratchBufferInArena = &RequestScratchBufferInArena;
  ctx.GetScratchBuffer = &GetScratchBuffer;
  ctx.tensors = tflTensors;
  ctx.tensors_size = 11;
  for(size_t i = 0; i < 11; ++i) {
    tflTensors[i].type = kTfLiteFloat32;
    tflTensors[i].is_variable = 0;

#if defined(EI_CLASSIFIER_ALLOCATION_HEAP)
    tflTensors[i].allocation_type = tensorData[i].allocation_type;
#else
    tflTensors[i].allocation_type = (tensor_arena <= tensorData[i].data && tensorData[i].data < tensor_arena + kTensorArenaSize) ? kTfLiteArenaRw : kTfLiteMmapRo;
#endif
    tflTensors[i].bytes = tensorData[i].bytes;
    tflTensors[i].dims = tensorData[i].dims;

#if defined(EI_CLASSIFIER_ALLOCATION_HEAP)
    if(tflTensors[i].allocation_type == kTfLiteArenaRw){
      uint8_t* start = (uint8_t*) ((uintptr_t)tensorData[i].data + (uintptr_t) tensor_arena);

     tflTensors[i].data.data =  start;
    }
    else{
       tflTensors[i].data.data = tensorData[i].data;
    }
#else
    tflTensors[i].data.data = tensorData[i].data;
#endif // EI_CLASSIFIER_ALLOCATION_HEAP
    tflTensors[i].quantization.type = kTfLiteNoQuantization;
    if (tflTensors[i].allocation_type == kTfLiteArenaRw) {
      auto data_end_ptr = (uint8_t*)tflTensors[i].data.data + tensorData[i].bytes;
      if (data_end_ptr > tensor_boundary) {
        tensor_boundary = data_end_ptr;
      }
    }
  }
  if (tensor_boundary > current_location /* end of arena size */) {
    printf("ERR: tensor arena is too small, does not fit model - even without scratch buffers\n");
    return kTfLiteError;
  }
  registrations[OP_FULLY_CONNECTED] = *tflite::ops::micro::Register_FULLY_CONNECTED();
  registrations[OP_SOFTMAX] = *tflite::ops::micro::Register_SOFTMAX();

  for(size_t i = 0; i < 4; ++i) {
    tflNodes[i].inputs = nodeData[i].inputs;
    tflNodes[i].outputs = nodeData[i].outputs;
    tflNodes[i].builtin_data = nodeData[i].builtin_data;
    tflNodes[i].custom_initial_data = nullptr;
    tflNodes[i].custom_initial_data_size = 0;
    if (registrations[nodeData[i].used_op_index].init) {
      tflNodes[i].user_data = registrations[nodeData[i].used_op_index].init(&ctx, (const char*)tflNodes[i].builtin_data, 0);
    }
  }
  for(size_t i = 0; i < 4; ++i) {
    if (registrations[nodeData[i].used_op_index].prepare) {
      TfLiteStatus status = registrations[nodeData[i].used_op_index].prepare(&ctx, &tflNodes[i]);
      if (status != kTfLiteOk) {
        return status;
      }
    }
  }
  return kTfLiteOk;
}

static const int inTensorIndices[] = {
  0, 
};
TfLiteTensor* trained_model_input(int index) {
  return &ctx.tensors[inTensorIndices[index]];
}

static const int outTensorIndices[] = {
  10, 
};
TfLiteTensor* trained_model_output(int index) {
  return &ctx.tensors[outTensorIndices[index]];
}

TfLiteStatus trained_model_invoke() {
  for(size_t i = 0; i < 4; ++i) {
    TfLiteStatus status = registrations[nodeData[i].used_op_index].invoke(&ctx, &tflNodes[i]);

#if EI_CLASSIFIER_PRINT_STATE
    ei_printf("layer %lu\n", i);
    ei_printf("    inputs:\n");
    for (size_t ix = 0; ix < tflNodes[i].inputs->size; ix++) {
      auto d = tensorData[tflNodes[i].inputs->data[ix]];

      size_t data_ptr = (size_t)d.data;

      if (d.allocation_type == kTfLiteArenaRw) {
        data_ptr = (size_t)tensor_arena + data_ptr;
      }

      if (d.type == TfLiteType::kTfLiteInt8) {
        int8_t* data = (int8_t*)data_ptr;
        ei_printf("        %lu (%zu bytes, ptr=%p, alloc_type=%d, type=%d): ", ix, d.bytes, data, (int)d.allocation_type, (int)d.type);
        for (size_t jx = 0; jx < d.bytes; jx++) {
          ei_printf("%d ", data[jx]);
        }
      }
      else {
        float* data = (float*)data_ptr;
        ei_printf("        %lu (%zu bytes, ptr=%p, alloc_type=%d, type=%d): ", ix, d.bytes, data, (int)d.allocation_type, (int)d.type);
        for (size_t jx = 0; jx < d.bytes / 4; jx++) {
          ei_printf("%f ", data[jx]);
        }
      }
      ei_printf("\n");
    }
    ei_printf("\n");

    ei_printf("    outputs:\n");
    for (size_t ix = 0; ix < tflNodes[i].outputs->size; ix++) {
      auto d = tensorData[tflNodes[i].outputs->data[ix]];

      size_t data_ptr = (size_t)d.data;

      if (d.allocation_type == kTfLiteArenaRw) {
        data_ptr = (size_t)tensor_arena + data_ptr;
      }

      if (d.type == TfLiteType::kTfLiteInt8) {
        int8_t* data = (int8_t*)data_ptr;
        ei_printf("        %lu (%zu bytes, ptr=%p, alloc_type=%d, type=%d): ", ix, d.bytes, data, (int)d.allocation_type, (int)d.type);
        for (size_t jx = 0; jx < d.bytes; jx++) {
          ei_printf("%d ", data[jx]);
        }
      }
      else {
        float* data = (float*)data_ptr;
        ei_printf("        %lu (%zu bytes, ptr=%p, alloc_type=%d, type=%d): ", ix, d.bytes, data, (int)d.allocation_type, (int)d.type);
        for (size_t jx = 0; jx < d.bytes / 4; jx++) {
          ei_printf("%f ", data[jx]);
        }
      }
      ei_printf("\n");
    }
    ei_printf("\n");
#endif // EI_CLASSIFIER_PRINT_STATE

    if (status != kTfLiteOk) {
      return status;
    }
  }
  return kTfLiteOk;
}

TfLiteStatus trained_model_reset( void (*free_fnc)(void* ptr) ) {
#ifdef EI_CLASSIFIER_ALLOCATION_HEAP
  free_fnc(tensor_arena);
#endif
  scratch_buffers.clear();
  for (size_t ix = 0; ix < overflow_buffers.size(); ix++) {
    free(overflow_buffers[ix]);
  }
  overflow_buffers.clear();
  return kTfLiteOk;
}