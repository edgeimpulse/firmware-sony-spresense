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

#ifndef _EI_CLASSIFIER_ANOMALY_CLUSTERS_H_
#define _EI_CLASSIFIER_ANOMALY_CLUSTERS_H_

#include "edge-impulse-sdk/anomaly/anomaly.h"

// (before - mean) / scale
const float ei_classifier_anom_scale[EI_CLASSIFIER_ANOM_AXIS_SIZE] = { 0.2342509671085176, 0.1570719992187718, 0.10679067179882987 };
const float ei_classifier_anom_mean[EI_CLASSIFIER_ANOM_AXIS_SIZE] = { 0.21024881153126437, 0.14628955504286767, 0.1151182290999579 };

const ei_classifier_anom_cluster_t ei_classifier_anom_clusters[EI_CLASSIFIER_ANOM_CLUSTER_COUNT] = { { { -0.8934527635574341, -0.920492947101593, -1.0530786514282227 }, 0.45132620571317705 }
, { { -0.2846887409687042, -0.07676900923252106, 2.3378050327301025 }, 1.0384047665181002 }
, { { 0.15630429983139038, 2.722520351409912, 1.1331573724746704 }, 0.6366082946726457 }
, { { 1.4800630807876587, -0.4810178875923157, 0.36334866285324097 }, 0.5847320325536354 }
, { { -0.32593268156051636, 0.5568314790725708, 0.00018878096307162195 }, 0.4911571268359613 }
, { { 2.132824420928955, -0.08540046215057373, 0.6424986124038696 }, 0.3389498675376486 }
, { { 1.0526208877563477, 2.6019444465637207, 0.7755225300788879 }, 0.7239422923276432 }
, { { -0.5692387819290161, -0.3417707085609436, 1.406075358390808 }, 0.4454943765118434 }
, { { -0.6318459510803223, -0.348737895488739, -0.8223416805267334 }, 0.37292291472119343 }
, { { 1.5990177392959595, 0.36818280816078186, -0.09366372227668762 }, 0.4657035009058123 }
, { { 1.4420100450515747, 0.02734968811273575, -0.41958343982696533 }, 0.4662447535012249 }
, { { -0.2012510448694229, 1.4041725397109985, -0.03933540731668472 }, 0.5242459709675492 }
, { { -0.4001999497413635, -0.41708695888519287, 2.0489485263824463 }, 0.751013594740254 }
, { { -0.6277750134468079, -0.22515353560447693, -0.5862524509429932 }, 0.42157039033884086 }
, { { -0.6864659190177917, -0.5843217968940735, 0.9222880601882935 }, 0.5314365700892766 }
, { { 1.1606767177581787, -0.3124772608280182, -0.3785923719406128 }, 0.4327404244966922 }
, { { 1.8195523023605347, 0.529654324054718, 0.37106072902679443 }, 0.42338404185813017 }
, { { -0.2588914930820465, 2.928114175796509, -0.4059930741786957 }, 0.46685127935813614 }
, { { 2.1495468616485596, 0.0026783673092722893, 1.1736869812011719 }, 0.30883269363869975 }
, { { -0.46744248270988464, -0.06216738000512123, 1.1630350351333618 }, 0.6448069119989349 }
, { { -0.4789751172065735, 0.22003041207790375, -0.14071400463581085 }, 0.5617125697003549 }
, { { 1.5089335441589355, -0.35020509362220764, -0.37356290221214294 }, 0.5192953887568909 }
, { { -0.5161659121513367, 0.01919231191277504, 1.6303718090057373 }, 1.0933649883316443 }
, { { 1.0807716846466064, -0.2254909873008728, 0.4341294467449188 }, 0.4825218017607249 }
, { { -0.16105329990386963, 2.0281739234924316, 0.6099803447723389 }, 0.6516491843932533 }
, { { -0.6089665293693542, 0.8726818561553955, 0.42993107438087463 }, 0.3288560610811248 }
, { { 0.03328349068760872, 0.5577679872512817, 0.6784090995788574 }, 0.5506744495469184 }
, { { 1.9712985754013062, 0.14053061604499817, 0.26413941383361816 }, 0.43395231587782546 }
, { { 0.022036466747522354, 3.297818183898926, -0.29070383310317993 }, 0.6115728515299753 }
, { { 0.3507869839668274, 1.7250434160232544, 2.995760679244995 }, 1.1376574035684863 }
, { { -0.25121086835861206, 2.2419095039367676, -0.0033563284669071436 }, 0.4455988219646793 }
, { { -0.316534161567688, 0.03579114004969597, 3.002131223678589 }, 0.4242883176623495 }
};

#endif // _EI_CLASSIFIER_ANOMALY_CLUSTERS_H_