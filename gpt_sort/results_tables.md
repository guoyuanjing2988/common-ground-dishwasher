| goal | runs | items found | safe / not-safe correct | category correct | soil level correct (safe items) | greasy correct | program = rule | program = rule on GPT own soil labels | plan feasible | plates loaded by robot, seated (<15 deg) | GPT calls | GPT s (median plan call) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| en | 12 | 82/82 (100 %) | 82/82 (100 %) | 82/82 (100 %) | 38/52 (73 %) | 44/52 (85 %) | 8/12 (67 %) | 11/12 (92 %) | 12/12 (100 %) | 12/12 attempted (11 also < 10 deg) | 24 | 498.6 (28.5) |
| zh | 12 | 82/82 (100 %) | 82/82 (100 %) | 82/82 (100 %) | 38/52 (73 %) | 46/52 (88 %) | 8/12 (67 %) | 11/12 (92 %) | 12/12 (100 %) | 12/12 attempted (12 also < 10 deg) | 24 | 538.4 (33.5) |
| all | 24 | 164/164 (100 %) | 164/164 (100 %) | 164/164 (100 %) | 76/104 (73 %) | 90/104 (87 %) | 16/24 (67 %) | 22/24 (92 %) | 24/24 (100 %) | 24/24 attempted (23 also < 10 deg) | 48 | 1037.0 (31.8) |

| seed | goal | items | found | safe ok | category ok | program (GPT / rule) | plan feasible | robot loaded / seated | plate lean deg | to human | deferred | GPT calls | GPT s | sim s | wall s | max unplanned contact N |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 | en | 7 | 7 | 7 | 7 | eco / eco ✓ | yes | 1 / 1 | 9.05 | 4 | 0 | 2 | 43.7 | 176.1 | 99.7 | 0.0 |
| 1 | en | 7 | 7 | 7 | 7 | intensive / eco ✗ | yes | 1 / 1 | 10.45 | 3 | 0 | 2 | 36.4 | 178.8 | 83.7 | 0.0 |
| 2 | en | 6 | 6 | 6 | 6 | quick / quick ✓ | yes | 1 / 1 | 9.42 | 2 | 0 | 2 | 39.6 | 177.8 | 79.6 | 0.0 |
| 3 | en | 6 | 6 | 6 | 6 | quick / quick ✓ | yes | 1 / 1 | 9.06 | 2 | 0 | 2 | 32.6 | 176.6 | 72.3 | 0.0 |
| 4 | en | 7 | 7 | 7 | 7 | intensive / intensive ✓ | yes | 1 / 1 | 9.08 | 4 | 0 | 2 | 42.2 | 176.7 | 99.3 | 0.0 |
| 5 | en | 8 | 8 | 8 | 8 | intensive / intensive ✓ | yes | 1 / 1 | 8.5 | 4 | 0 | 2 | 40.2 | 177.9 | 83.4 | 0.0 |
| 6 | en | 7 | 7 | 7 | 7 | intensive / eco ✗ | yes | 1 / 1 | 9.05 | 4 | 0 | 2 | 53.8 | 177.4 | 111.6 | 0.0 |
| 7 | en | 7 | 7 | 7 | 7 | eco / eco ✓ | yes | 1 / 1 | 9.16 | 4 | 0 | 2 | 36.9 | 177.7 | 99.1 | 0.0 |
| 8 | en | 6 | 6 | 6 | 6 | quick / eco ✗ | yes | 1 / 1 | 8.68 | 3 | 0 | 2 | 36.3 | 176.0 | 75.7 | 0.0 |
| 9 | en | 6 | 6 | 6 | 6 | intensive / intensive ✓ | yes | 1 / 1 | 9.53 | 3 | 0 | 2 | 41.9 | 178.0 | 90.7 | 0.0 |
| 10 | en | 9 | 9 | 9 | 9 | intensive / intensive ✓ | yes | 1 / 1 | 8.69 | 4 | 1 | 2 | 50.4 | 179.6 | 102.6 | 0.0 |
| 11 | en | 6 | 6 | 6 | 6 | eco / quick ✗ | yes | 1 / 1 | 8.99 | 2 | 0 | 2 | 44.6 | 178.1 | 86.2 | 0.0 |
| 0 | zh | 7 | 7 | 7 | 7 | eco / eco ✓ | yes | 1 / 1 | 9.04 | 4 | 0 | 2 | 45.4 | 176.2 | 102.5 | 0.0 |
| 1 | zh | 7 | 7 | 7 | 7 | intensive / eco ✗ | yes | 1 / 1 | 8.52 | 3 | 0 | 2 | 46.1 | 178.8 | 96.0 | 0.0 |
| 2 | zh | 6 | 6 | 6 | 6 | quick / quick ✓ | yes | 1 / 1 | 9.01 | 2 | 0 | 2 | 36.3 | 177.8 | 76.8 | 0.0 |
| 3 | zh | 6 | 6 | 6 | 6 | eco / quick ✗ | yes | 1 / 1 | 9.1 | 2 | 0 | 2 | 41.8 | 177.6 | 81.8 | 0.0 |
| 4 | zh | 7 | 7 | 7 | 7 | intensive / intensive ✓ | yes | 1 / 1 | 9.16 | 4 | 0 | 2 | 40.0 | 176.7 | 95.9 | 0.0 |
| 5 | zh | 8 | 8 | 8 | 8 | intensive / intensive ✓ | yes | 1 / 1 | 8.45 | 4 | 0 | 2 | 42.2 | 177.8 | 84.3 | 0.0 |
| 6 | zh | 7 | 7 | 7 | 7 | eco / eco ✓ | yes | 1 / 1 | 9.07 | 4 | 0 | 2 | 47.2 | 178.0 | 104.2 | 0.0 |
| 7 | zh | 7 | 7 | 7 | 7 | eco / eco ✓ | yes | 1 / 1 | 9.14 | 4 | 0 | 2 | 65.4 | 177.7 | 125.5 | 0.0 |
| 8 | zh | 6 | 6 | 6 | 6 | quick / eco ✗ | yes | 1 / 1 | 8.81 | 3 | 0 | 2 | 39.9 | 176.0 | 79.1 | 0.0 |
| 9 | zh | 6 | 6 | 6 | 6 | intensive / intensive ✓ | yes | 1 / 1 | 9.11 | 3 | 0 | 2 | 39.2 | 178.1 | 86.7 | 0.0 |
| 10 | zh | 9 | 9 | 9 | 9 | intensive / intensive ✓ | yes | 1 / 1 | 8.74 | 4 | 1 | 2 | 51.8 | 179.5 | 103.3 | 0.0 |
| 11 | zh | 6 | 6 | 6 | 6 | eco / quick ✗ | yes | 1 / 1 | 9.09 | 2 | 0 | 2 | 43.1 | 178.2 | 82.7 | 0.0 |

| true type | items | safe/not-safe correct | category correct |
|---|---|---|---|
| board | 20 | 20/20 (100 %) | 20/20 (100 %) |
| bowl | 14 | 14/14 (100 %) | 14/14 (100 %) |
| flask | 10 | 10/10 (100 %) | 10/10 (100 %) |
| knife | 12 | 12/12 (100 %) | 12/12 (100 %) |
| mug | 34 | 34/34 (100 %) | 34/34 (100 %) |
| pan | 18 | 18/18 (100 %) | 18/18 (100 %) |
| plate | 56 | 56/56 (100 %) | 56/56 (100 %) |

Failures / deviations:
- s1 en: program intensive vs ground truth eco (rule on GPT's own labels: intensive; label errors in the load: plate soil light->medium, greasy False->True; plate soil light->medium)
- s6 en: program intensive vs ground truth eco (rule on GPT's own labels: intensive; label errors in the load: plate soil medium->light; bowl soil medium->heavy, greasy False->True)
- s8 en: program quick vs ground truth eco (rule on GPT's own labels: eco; label errors in the load: none)
- s11 en: program eco vs ground truth quick (rule on GPT's own labels: eco; label errors in the load: mug soil light->medium)
- s1 zh: program intensive vs ground truth eco (rule on GPT's own labels: intensive; label errors in the load: plate soil light->medium, greasy False->True)
- s3 zh: program eco vs ground truth quick (rule on GPT's own labels: eco; label errors in the load: plate soil light->medium)
- s8 zh: program quick vs ground truth eco (rule on GPT's own labels: eco; label errors in the load: none)
- s11 zh: program eco vs ground truth quick (rule on GPT's own labels: eco; label errors in the load: mug soil light->medium)
