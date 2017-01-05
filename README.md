# brandes-betweenness-centrality
Multithreaded implementation of Brandes algorithm founded on pseudo-code provided by our teacher:

```
for each v : V 

   BC[v] = 0;

for each s : V { // in parallel

   S = stack();

   for all w in V {

      P[w] = list();

      sigma[w] = 0;

      d[w] = -1;

      delta[v] = 0;

   }

   sigma[s] = 1;

   d[s] = 0;

   Q = queue(); // FIFO

   Q.push_back(s);

   while (!Q.empty()) {

      v = Q.pop_front();

      S.push(v);

      for each neighbor w of v {

         if d[w] < 0 {

            Q.push_back(w);

            d[w] = d[v] + 1;

         }

         if (d[w] == d[v] + 1) {

            sigma[w] += sigma[v];

            P[w].append(v);

         }

      }

   }

   while (!S.empty()) {

     w = S.pop();

     for each v in P[w]

        delta[v] += (sigma[v] / sigma[w])(1 + delta[w]);

     if (w != s)

        BC[w] += delta[w];

   }

}
```
