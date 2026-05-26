#ifndef HARM_UTIL_H
#define HARM_UTIL_H

namespace ns3 {

/**
 * Pairwise HARM metric, formula (3) of Sidorenko et al. (VTC2023-Fall):
 *     H_{1,2} = m2 / (m1 + m2) * |v1 - v2|
 *
 * Asymmetric: it returns the harm to vehicle 1 caused by a hypothetical
 * collision with vehicle 2. For the time-sampled HARM log we compute it
 * for each unordered pair once, so the convention there is to use the
 * pair as (i, j) with i < j and report only one row.
 *
 * If the two masses sum to a non-positive value the function returns 0.
 */
double appUtil_pairwiseHarm (double m1, double v1, double m2, double v2);

} // namespace ns3

#endif /* HARM_UTIL_H */
