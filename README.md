# Pendolo di Furuta — modello non lineare, simulazione e controllo

Questo repository raccoglie il codice C++17 e gli strumenti di sviluppo per un **pendolo di Furuta**: un sistema meccanico *sottattuato* (due gradi di libertà, un solo attuatore), fortemente non lineare, usato come banco di prova per tecniche di controllo. L'obiettivo del progetto è portare il pendolo dalla posizione di riposo (appeso) alla verticale tramite una legge di **swing-up a energia** e, in prospettiva, mantenerlo in equilibrio con un controllore di stabilizzazione.

> **Stato del progetto.** Alcune parti descritte in questo README — il modello dinamico in forma matriciale, il solutore Runge-Kutta 4, la legge di swing-up completa, la saturazione e la mappatura su tensione — sono **design previsto e non ancora implementate**. La tabella in [§11](#11-stato-dellimplementazione-e-roadmap) distingue in modo esplicito ciò che è operativo da ciò che è pianificato. Il resto (parametri, filtri di velocità, calcolo dell'energia, stima di $k_t$ e $k_V$) è codice reale in questo repository.

## Indice

- [1. Il sistema fisico](#1-il-sistema-fisico)
- [2. Hardware e catena di attuazione](#2-hardware-e-catena-di-attuazione)
- [3. Convenzioni, simboli e parametri](#3-convenzioni-simboli-e-parametri)
- [4. Modello dinamico non lineare](#4-modello-dinamico-non-lineare)
- [5. Integrazione numerica: Runge-Kutta 4](#5-integrazione-numerica-runge-kutta-4)
- [6. Stima della velocità dagli AS5600](#6-stima-della-velocità-dagli-as5600)
- [7. Controllo di swing-up a energia](#7-controllo-di-swing-up-a-energia)
- [8. Dall'accelerazione comandata al motore](#8-dallaccelerazione-comandata-al-motore)
- [9. Architettura software](#9-architettura-software)
- [10. Build ed esecuzione](#10-build-ed-esecuzione)
- [11. Stato dell'implementazione e roadmap](#11-stato-dellimplementazione-e-roadmap)
- [12. Riferimenti](#12-riferimenti)

---

## 1. Il sistema fisico

Il pendolo di Furuta (o *rotational inverted pendulum*) è composto da:

- un **braccio** (nel codice `PinwheelParameters`, massa $m_1$) che ruota in un piano orizzontale attorno all'asse di un motore;
- un **pendolo** (massa $m_2$) incernierato all'estremità del braccio, libero di ruotare in un piano verticale.

Il sistema ha due gradi di libertà, $\theta$ (braccio) e $\alpha$ (pendolo), ma un solo attuatore: la coppia $\tau_1$ del motore agisce solo su $\theta$. La cerniera del pendolo è **passiva** ($\tau_2 = 0$): è questo che rende il sistema *underactuated*.

Due equilibri notevoli:

| Configurazione | $\alpha$ | Stabilità | Energia del pendolo |
|---|---|---|---|
| pendolo appeso (riposo) | $0$ | stabile | $E = 0$ |
| pendolo verticale (in piedi) | $\pi$ | instabile | $E = E^\star = 2m_2gl_2$ |

La nonlinearità nasce da tre sorgenti:

1. la **gravità**, che genera una coppia $\propto \sin\alpha$ sulla cerniera;
2. i termini **centrifughi e di Coriolis**, che accoppiano $\dot\theta$ e $\dot\alpha$ (termini $\propto \sin 2\alpha$ e $\propto \sin\alpha\,\dot\alpha^2$);
3. l'**inerzia del braccio vista dal motore**, che dipende dalla posizione del pendolo: $\hat J_0 + \hat J_2\sin^2\alpha$.

Lo schema di riferimento è la Fig. 1 della pagina Wikipedia [Furuta pendulum](https://en.wikipedia.org/wiki/Furuta_pendulum), da cui il progetto ha ripreso geometria e convenzioni.

---

## 2. Hardware e catena di attuazione

I limiti dell'attuatore sono raccolti in `ActuatorLimits` (`furuta/include/furuta/parameters.hpp`):

| Parametro | Valore | Note |
|---|---|---|
| `maxRPM` | 3850 | regime massimo dell'asse, $\approx 403\ \mathrm{rad/s}$ |
| `voltage` | 24 V | tensione di alimentazione |
| `poles` | 8 | coppie polari motore BLDC — **non usato oggi** |
| `nPower` | 50 W | potenza nominale |
| `nRPM` | 3000 | velocità nominale |
| `maxTorque`, `maxCurrent` | *(vuoti)* | da identificare/dichiarare |
| `mode` | 0 | selettore delle formule di $k_t, k_V$ (vedi [§8](#8-dallaccelerazione-comandata-al-motore)) |

La misura degli angoli è affidata a **due encoder magnetici AS5600** (12 bit assoluti, 4096 conteggi/giro, $1\ \mathrm{LSB} = 2\pi/4096 \approx 1.53\ \mathrm{mrad}$), uno sul braccio e uno sulla cerniera del pendolo, letti via I2C.

Stato attuale della catena:

- **oggi** il codice gira su host PC e `main.cpp` sostituisce i sensori con un *mock* (`getAngles(t)`: $\theta = 0.3\sin(0.5t)$, $\alpha = 0.8\sin(2t)$);
- **previsto** il porting su board con driver [SimpleFOC](https://simplefoc.com/) per il motore e driver I2C per gli AS5600 (il commento in `math_utils.hpp` cita già la velocità stimata da SimpleFOC).

---

## 3. Convenzioni, simboli e parametri

### 3.1 Convenzioni angolari

Le convenzioni sono quelle di [Cazzolato & Prime (2011)](https://downloads.hindawi.com/journals/jcse/2011/528341.pdf), riprese da Wikipedia e già coerenti con il codice:

- $\theta$ (braccio): angolo nel piano orizzontale, positivo **antiorario visto dall'alto**;
- $\alpha$ (pendolo): angolo nel piano verticale, positivo **antiorario visto di fronte**, con $\alpha = 0$ quando il pendolo è appeso verso il basso e $\alpha = \pi$ quando è verticale in alto;
- $\tau_1$: coppia motrice, positiva concorde a $\theta$ (antioraria vista dall'alto);
- $\tau_2 = 0$: la cerniera del pendolo non è attuata.

Il codice è internamente coerente con questa scelta: l'energia implementata in `energy_swing_up.hpp`, $E = \frac12 \hat J_2\dot\alpha^2 + m_2gl_2(1-\cos\alpha)$ (con $\hat J_2$ = `J_yy`), vale $0$ da fermo in basso ($\alpha = 0$) e $E^\star = 2m_2gl_2$ da fermo in alto ($\alpha = \pi$).

### 3.2 Tabella dei simboli

| Simbolo qui | Codice (`parameters.hpp` / altro) | Wikipedia/Cazzolato | Significato |
|---|---|---|---|
| $m_1$ | `PinwheelParameters::pinwheelMass` | $m_1$ | massa del braccio |
| $l_1$ | `PinwheelParameters::pinwheelToCoM` | $l_1$ | distanza asse motore → CoM del braccio |
| $L_1$ | `PinwheelParameters::pendulumToCenter` | $L_1$ | distanza asse motore → cerniera del pendolo (il nome nel codice è fuorviante) |
| $m_2$ | `ArmParameters::pendulumMass` | $m_2$ | massa del pendolo |
| $l_2$ | `ArmParameters::pendulumToCoM` | $l_2$ | distanza cerniera → CoM del pendolo |
| $L_2$ | `ArmParameters::pendulumLenght` | $L_2$ | lunghezza del pendolo (refuso `Lenght` nel codice) |
| $\hat J_2$ | `ArmParameters::J_yy` | $\hat J_2 = J_{2zz} + m_2l_2^2$ | inerzia del pendolo rispetto alla **cerniera** (già comprensiva del trasporto) |
| $J_{2xx}$ | `ArmParameters::J_xx` | $J_{2xx}$ | componente trasversale del tensore d'inerzia del pendolo — **non usata oggi** |
| $\hat J_0$ | *(assente)* | $\hat J_0 = J_1 + m_1l_1^2 + m_2L_1^2$ | inerzia totale vista dal motore con pendolo in basso — **da completare: manca $J_1$** |
| $b_1, b_2$ | *(assenti)* | $b_1, b_2$ | attriti viscosi (motore e cerniera) — **non identificati** |
| $\mu$ | `mu` | — | costante dichiarata ma **mai usata**; semantica da chiarire (candidato: attrito) |
| $g$ | `gravity` | $g$ | accelerazione di gravità |

### 3.3 Parametri numerici

Geometria e inerzie provengono dal CAD (`dimensions.txt`: J1 = braccio, J2 = pendolo, origine di J2 sulla cerniera).

| Parametro | Valore | Unità |
|---|---|---|
| $m_1$ | 0.096567 | kg |
| $l_1$ | 0.0070202 | m |
| $L_1$ | 0.079073 | m |
| $m_2$ | 0.021332 | kg |
| $l_2$ | 0.066611 | m |
| $L_2$ | 0.0859 | m |
| $\hat J_2$ (`J_yy`) | $1.0513\times10^{-4}$ | kg·m² |
| $J_{2xx}$ (`J_xx`) | $9.9256\times10^{-7}$ | kg·m² |
| $g$ | 9.81 | m/s² |

Per $L_2$: l'altezza del bounding box CAD del pendolo è 97.9 mm, di cui 12 mm di materiale di accoppiamento sopra la cerniera.

Grandezze derivate utili:

$$m_2gl_2 = 1.394\times10^{-2}\ \mathrm{J}, \qquad E^\star = 2m_2gl_2 = 2.788\times10^{-2}\ \mathrm{J}, \qquad m_2L_1l_2 = 1.124\times10^{-4}\ \mathrm{kg\,m^2}$$

$$\dot\theta_{max} = 2\pi\frac{3850}{60} \approx 403\ \mathrm{rad/s}$$

**Incertezze note da chiarire** (finiscono nella roadmap):

- $\hat J_2$ è il valore CAD (trasporto alla cerniera incluso): è circa il doppio della stima di un'asta uniforme ($m_2L_2^2/3 \approx 5.25\times10^{-5}\ \mathrm{kg\,m^2}$), coerente con la massa concentrata verso la punta ($l_2/L_2 \approx 0.78$);
- `J_xx` è il momento rispetto all'asse longitudinale del pendolo ($\approx 0$ per un pendolo snello) e non entra né nel modello semplificato né nel codice attuale;
- `mu` è dichiarata ma non compare in nessuna formula;
- $\hat J_0$ non è calcolabile finché non si dispone di $J_1$ o di una misura complessiva dell'inerzia vista dal motore.

---

## 4. Modello dinamico non lineare

### 4.1 Derivazione lagrangiana (forma compatta)

Indicando $q = [\theta,\ \alpha]^T$, la lagrangiana è $L = T - V$ con (ipotesi di pendolo snello e simmetrico, vedi [§4.3](#43-modello-semplificato-lumped)):

$$T = \tfrac12\left(\hat J_0 + \hat J_2\sin^2\alpha\right)\dot\theta^2 + \tfrac12\hat J_2\dot\alpha^2 + m_2L_1l_2\cos\alpha\,\dot\theta\dot\alpha$$

$$V = m_2gl_2\left(1-\cos\alpha\right)$$

dove $V$ è scelto con zero nella posizione appesa (coerente con $E$ del codice). Le forze generalizzate sono $Q_\theta = \tau_1 - b_1\dot\theta$ e $Q_\alpha = -b_2\dot\alpha$. Le equazioni di Eulero-Lagrange

$$\frac{d}{dt}\frac{\partial L}{\partial \dot q_i} - \frac{\partial L}{\partial q_i} = Q_i$$

danno il sistema non lineare completo del Furuta pendulum. La derivazione estesa è nel paper di [Cazzolato & Prime (2011)](https://downloads.hindawi.com/journals/jcse/2011/528341.pdf); qui riportiamo il risultato.

### 4.2 Equazioni complete

Con i simboli di Wikipedia ($\theta_1 \equiv \theta$, $\theta_2 \equiv \alpha$) e i tre termini inerziali del pendolo $J_{2xx}, J_{2yy}, J_{2zz}$:

$$
\begin{aligned}
\ddot\theta\Big(J_{1zz} + m_1l_1^2 + m_2L_1^2 + (J_{2yy} + m_2l_2^2)\sin^2\alpha + J_{2xx}\cos^2\alpha\Big)
&+ \ddot\alpha\, m_2L_1l_2\cos\alpha
- m_2L_1l_2\sin\alpha\,\dot\alpha^2 \\
&+ \dot\theta\dot\alpha\sin(2\alpha)\left(m_2l_2^2 + J_{2yy} - J_{2xx}\right)
+ b_1\dot\theta = \tau_1
\end{aligned}
$$

$$
\begin{aligned}
\ddot\theta\, m_2L_1l_2\cos\alpha
&+ \ddot\alpha\left(m_2l_2^2 + J_{2zz}\right)
+ \tfrac12\dot\theta^2\sin(2\alpha)\left(-m_2l_2^2 - J_{2yy} + J_{2xx}\right) \\
&+ b_2\dot\alpha + gm_2l_2\sin\alpha = \tau_2
\end{aligned}
$$

### 4.3 Modello semplificato (*lumped*)

La maggior parte dei Furuta pendulum ha bracci lunghi e sottili: l'inerzia attorno all'asse del pendolo è trascurabile e il pendolo ha simmetria rotazionale. Sotto queste ipotesi ($J_{2xx}\to 0$, $J_{2yy} = J_{2zz} = J_2$) si definiscono:

$$\hat J_0 = J_1 + m_1l_1^2 + m_2L_1^2, \qquad \hat J_2 = J_2 + m_2l_2^2$$

e le equazioni diventano:

$$
\ddot\theta\left(\hat J_0 + \hat J_2\sin^2\alpha\right)
+ \ddot\alpha\, m_2L_1l_2\cos\alpha
- m_2L_1l_2\sin\alpha\,\dot\alpha^2
+ \dot\theta\dot\alpha\sin(2\alpha)\,\hat J_2
+ b_1\dot\theta = \tau_1
$$

$$
\ddot\theta\, m_2L_1l_2\cos\alpha
+ \ddot\alpha\,\hat J_2
- \tfrac12\dot\theta^2\sin(2\alpha)\,\hat J_2
+ b_2\dot\alpha
+ gm_2l_2\sin\alpha = \tau_2 = 0
$$

**È questa la forma che il codice adotta** (`J_yy` $\equiv \hat J_2$); le equazioni complete restano come riferimento per una modellazione più fine (dove `J_xx` entrerebbe come $J_{2xx}$).

### 4.4 Forma matriciale per l'integratore

Riscrivendo il modello semplificato come $M(q)\ddot q + h(q,\dot q) + G(q) = \tau$ con $\tau = [\tau_1,\ 0]^T$:

$$
M(q) =
\begin{bmatrix}
\hat J_0 + \hat J_2\sin^2\alpha & m_2L_1l_2\cos\alpha \\
m_2L_1l_2\cos\alpha & \hat J_2
\end{bmatrix},
\qquad
G(q) =
\begin{bmatrix}
0 \\
m_2gl_2\sin\alpha
\end{bmatrix}
$$

$$
h(q,\dot q) =
\begin{bmatrix}
-\,m_2L_1l_2\sin\alpha\,\dot\alpha^2 + \hat J_2\sin(2\alpha)\,\dot\theta\dot\alpha + b_1\dot\theta \\
-\,\tfrac12\hat J_2\sin(2\alpha)\,\dot\theta^2 + b_2\dot\alpha
\end{bmatrix}
$$

La decomposizione di $h$ in $C(q,\dot q)\dot q$ **non è unica** (dipende dalla scelta dei simboli di Christoffel); ai fini dell'integrazione numerica è sufficiente avere $h$ in forma esplicita, perché il solutore risolve

$$\ddot q = M(q)^{-1}\left(\tau - h(q,\dot q) - G(q)\right)$$

e l'inversa di una $2\times2$ è banale: $\det M = M_{11}M_{22} - M_{12}^2$. Oggi $h$ e $G$ non sono implementate, e $\hat J_0$ è incompleta: vedi [§11](#11-stato-dellimplementazione-e-roadmap).

---

## 5. Integrazione numerica: Runge-Kutta 4

### 5.1 Perché RK4

Il modello è un sistema di ODE del primo ordine nello stato $x = [\theta,\ \alpha,\ \dot\theta,\ \dot\alpha]^T$:

$$\dot x = f(x, u), \qquad f(x,u) = \begin{bmatrix}\dot\theta \\ \dot\alpha \\ \ddot\theta \\ \ddot\alpha\end{bmatrix}, \qquad u = \tau_1$$

dove $\ddot q$ si ottiene come in [§4.4](#44-forma-matriciale-per-lintegratore). RK4 è un metodo **esplicito a passo fisso**, del quarto ordine: senza Jacobiane e senza passi adattativi, è adatto a girare su MCU con costo per passo contenuto (4 valutazioni di $f$ per passo). L'alternativa minima (Eulero esplicito) è del primo ordine e su questo sistema introduce rapidamente deriva di energia; i metodi a passo variabile (RK45/Dormand-Prince) non sono necessari se il passo è scelto bene.

### 5.2 Algoritmo

Per il sistema autonomo $\dot x = f(x)$ con passo $h$:

$$
\begin{aligned}
k_1 &= f(x_n) \\
k_2 &= f\!\left(x_n + \tfrac{h}{2}k_1\right) \\
k_3 &= f\!\left(x_n + \tfrac{h}{2}k_2\right) \\
k_4 &= f\!\left(x_n + h\,k_3\right) \\
x_{n+1} &= x_n + \frac{h}{6}\left(k_1 + 2k_2 + 2k_3 + k_4\right)
\end{aligned}
$$

Tableau di Butcher: $c = [0,\ \tfrac12,\ \tfrac12,\ 1]$, pesi $b = [\tfrac16,\ \tfrac13,\ \tfrac13,\ \tfrac16]$.

Errore di troncamento locale $O(h^5)$, errore globale $O(h^4)$. La regione di stabilità sull'asse reale è circa $h\lambda \in (-2.78,\ 0)$: per un sistema con modo più veloce $\lambda_{max}$, il vincolo pratico è $h \lesssim 2.78/|\lambda_{max}|$.

### 5.3 Pseudocodice e aggancio al modello

```cpp
// Una valutazione di xdot = f(x, tau).
State stateDerivative(const State& x, double tau) {
    // q = [theta, alpha], qd = [thetaDot, alphaDot]
    const Mat2 M   = massMatrix(q);                      // §4.4
    const Vec2 h   = coriolisCentrifugal(q, qd);          // §4.4
    const Vec2 G   = gravity(q);                          // §4.4
    const Vec2 tauVec{tau, 0.0};                          // cerniera passiva
    const Vec2 qdd = solve2x2(M, tauVec - h - G);
    return { qd[0], qd[1], qdd[0], qdd[1] };
}

// Un passo di RK4 a coppia costante (zero-order hold).
State rk4Step(const State& x, double tau, double h) {
    const State k1 = stateDerivative(x, tau);
    const State k2 = stateDerivative(x + (h / 2.0) * k1, tau);
    const State k3 = stateDerivative(x + (h / 2.0) * k2, tau);
    const State k4 = stateDerivative(x + h * k3, tau);
    return x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}
```

### 5.4 Scelta del passo $h$

Assunzione di progetto: **$h = 1\ \mathrm{ms}$ (1 kHz)**, con coppia mantenuta costante tra due passi (ZOH). I tempi scala del sistema giustificano la scelta:

- modo pendolare: $\omega_0 = \sqrt{m_2gl_2/\hat J_2} \approx 11.5\ \mathrm{rad/s}$ ($\approx 0.55$ s di periodo);
- braccio a regime massimo: $403\ \mathrm{rad/s}$ ($\approx 16$ ms/giro).

Con $h = 1$ ms il transitorio più veloce è campionato con ampio margine. Note operative:

- se il loop di controllo girasse più lento di $h$, si possono fare **sottopassi di RK4** dentro un singolo periodo di controllo;
- lo stato integrato contiene $\alpha$ **non wrappato**: il pendolo può ruotare su più giri e il wrapping a $(-\pi,\pi]$ si applica solo alle letture dei sensori (vedi [§6](#6-stima-della-velocità-dagli-as5600));
- il valore di $h$ e la frequenza reale del loop vanno verificati sul target.

---

## 6. Stima della velocità dagli AS5600

Il codice (`velocity_filters.hpp`) stima $\dot\theta$ e $\dot\alpha$ da due letture **quantizzate**: $1\ \mathrm{LSB} = 2\pi/4096\ \mathrm{rad}$ e derivata numerica grezza amplificano il rumore di $1/\Delta t$. Sono disponibili tre stimatori, selezionabili con l'enum `VelocityFilterMode`; un'istanza per sensore (una per il braccio, una per il pendolo).

### 6.1 Wrapping

L'AS5600 è assoluto su un solo giro: agli estremi la lettura salta da $\approx 2\pi$ a $0$. Il salto viene riportato in $(-\pi,\pi]$ prima di ogni aggiornamento:

$$\Delta\theta_{w} = \operatorname{wrap}_{\pi}(\theta_k - \theta_{k-1})$$

### 6.2 Low-pass sulla derivata grezza (`LowPass`)

$$v_k^{raw} = \frac{\Delta\theta_w}{\Delta t}, \qquad RC = \frac{1}{2\pi f_c}, \qquad \beta = \frac{\Delta t}{RC + \Delta t}, \qquad v_k = v_{k-1} + \beta\left(v_k^{raw} - v_{k-1}\right)$$

L'EMA ha guadagno unitario in continua: una velocità costante non viene attenuata; viene filtrato solo il rumore. $f_c$ è la frequenza di taglio a $-3\ \mathrm{dB}$. La forma incrementale è numericamente più stabile della forma equivalente $v_k = \beta v_k^{raw} + (1-\beta)v_{k-1}$ quando $\beta$ è piccolo (nessuna cancellazione catastrofica).

### 6.3 Filtro di Kalman (`Kalman`)

Modello a velocità costante, stato $[\theta,\ \omega]^T$:

- predizione: $\hat x^- = F\hat x$, $P^- = FPF^T + Q$, con $F = \begin{bmatrix}1 & \Delta t \\ 0 & 1\end{bmatrix}$ e $Q = \mathrm{diag}(q_\theta, q_\omega)$;
- update: innovazione *wrappata* $r = \operatorname{wrap}_\pi(\theta_m - H\hat x^-)$, $H = [1\ 0]$, guadagno $K = P^-H^T(HP^-H^T + R)^{-1}$, quindi $\hat x = \hat x^- + Kr$ e $P = (I-KH)P^-$.

La misura di rumore è ricavata dalla quantizzazione, $R = \mathrm{LSB}^2/12$ (varianza uniforme su un LSB), mentre $q_\theta, q_\omega$ sono rumore di processo da tarare. La velocità è uno *stato* del filtro, non una derivata: per questo è meno rumorosa a parità di ritardo.

### 6.4 Filtro $\alpha$-$\beta$ (`AlphaBeta`)

Stesso modello del Kalman, con guadagni fissi:

$$\hat\theta = \theta + \omega\Delta t, \qquad r = \operatorname{wrap}_\pi(\theta_m - \hat\theta), \qquad \theta \leftarrow \hat\theta + \alpha_{ab}\,r, \qquad \omega \leftarrow \omega + \frac{\beta_{ab}}{\Delta t}\,r$$

Regole di taratura: $0 < \alpha_{ab} < 2$, $0 < \beta_{ab} < 4 - 2\alpha_{ab}$; la scelta $\beta_{ab} = \alpha_{ab}^2/(2-\alpha_{ab})$ rende la risposta criticamente smorzata. I default del codice sono $\alpha_{ab} = 0.5$, $\beta_{ab} = 0.1667 = 0.5^2/1.5$, cioè il caso critico.

Tutti gli stimatori espongono `reset(angle_rad)` per agganciare lo stato senza spike (dopo l'homing o un reset del loop) e restituiscono $0$ al primo campione.

---

## 7. Controllo di swing-up a energia

### 7.1 Energia del pendolo

Implementata in `energy_swing_up.hpp`:

$$E = \tfrac12 \hat J_2\,\dot\alpha^2 + m_2gl_2\left(1-\cos\alpha\right), \qquad E^\star = 2m_2gl_2, \qquad \mathrm{errE} = E^\star - E$$

$E$ è zero con il pendolo fermo in basso e vale $E^\star$ con il pendolo fermo in alto. L'obiettivo dello swing-up è portare $\mathrm{errE} \to 0$ pompando energia attraverso il braccio.

### 7.2 Perché funziona: l'accoppiamento energia ↔ accelerazione del motore

Il termine di accoppiamento $m_2L_1l_2\cos\alpha\,\ddot\theta$ nell'equazione del pendolo è la firma fisica di un fatto semplice: quando il braccio accelera, la cerniera del pendolo subisce un'accelerazione tangenziale $L_1\ddot\theta$, che compie lavoro sul pendolo. La potenza scambiata è proporzionale a $\cos\alpha$ e al verso di $\dot\alpha$: accelerare il braccio *in fase* col moto del pendolo ne aumenta l'ampiezza di oscillazione.

Derivazione breve (trascurando $b_2$ e con $\tau_2 = 0$), dall'equazione del pendolo in [§4.3](#43-modello-semplificato-lumped):

$$\hat J_2\ddot\alpha = -m_2L_1l_2\cos\alpha\,\ddot\theta + \tfrac12\hat J_2\dot\theta^2\sin(2\alpha) - m_2gl_2\sin\alpha$$

Sostituendo in $\dot E = \dot\alpha\left(\hat J_2\ddot\alpha + m_2gl_2\sin\alpha\right)$:

$$\dot E = -m_2L_1l_2\cos\alpha\,\ddot\theta\,\dot\alpha + \tfrac12\hat J_2\dot\theta^2\sin(2\alpha)\,\dot\alpha$$

Scegliendo la **legge di swing-up**

$$\boxed{\ \ddot\theta_{cmd} = k \cdot \mathrm{errE} \cdot \dot\alpha \cdot \cos\alpha\ }$$

si ottiene

$$\dot E = -m_2L_1l_2\,k\,\mathrm{errE}\,\dot\alpha^2\cos^2\alpha + \tfrac12\hat J_2\dot\theta^2\sin(2\alpha)\,\dot\alpha$$

Il primo termine è $\ge 0$ finché $E < E^\star$ (cioè $\mathrm{errE} > 0$): l'energia del pendolo cresce e si annulla esattamente a $E = E^\star$. Le convenzioni adottate in questo progetto ($\alpha$ misurato dal basso, $\mathrm{errE} = E^\star - E$) **non richiedono cambi di segno** rispetto alla formula classica.

Caveat noti:

- il secondo termine dipende da $\dot\theta^2$ e ha media nulla su un ciclo di pompaggio: non impedisce la crescita di $E$, ma una dimostrazione formale richiede di includere l'energia cinetica del braccio nella funzione di Lyapunov (o di usare l'energia totale);
- per $\cos\alpha = 0$ (pendolo orizzontale, $\alpha = \pm\pi/2$) la legge non pompa energia: è la *dead zone* fisiologica dello swing-up energetico;
- $\dot\alpha$ usato è `alphaDotFiltered`, non la derivata grezza: il ritardo del filtro va tenuto in conto nella taratura di $k$;
- in letteratura la variante con $\operatorname{sign}(\dot\alpha\cos\alpha)$ ha guadagno equivalente costante; la versione proporzionale qui scelta è più dolce vicino alla verticale.

### 7.3 Saturazione

$$\ddot\theta_{cmd} = \operatorname{clamp}\!\left(k\,\mathrm{errE}\,\dot\alpha\cos\alpha,\ -\ddot\theta_{max},\ +\ddot\theta_{max}\right)$$

con $\ddot\theta_{max}$ configurabile e, in prospettiva, derivato da coppia e inerzia come discusso in [§8](#8-dallaccelerazione-comandata-al-motore). Va ricordato che `maxRPM` limita $\dot\theta$ ($\approx 403\ \mathrm{rad/s}$), **non** $\ddot\theta$.

**Stato:** in `energy_swing_up.hpp` sono implementate $E$, $E^\star$ e $\mathrm{errE}$; la legge di controllo e la saturazione sono da implementare (`controlOutput` oggi è uno stub con il solo campo `velocity`).

---

## 8. Dall'accelerazione comandata al motore

La legge di swing-up produce un comando in **accelerazione angolare**. Il passaggio a tensione è lasciato esplicitamente aperto; le due vie candidate sono:

**Percorso A — torque path (preferito).** Sfrutta le grandezze già presenti:

1. coppia stimata: $\tau_1 \approx M_{11}(\alpha)\,\ddot\theta_{cmd} = (\hat J_0 + \hat J_2\sin^2\alpha)\,\ddot\theta_{cmd}$ (il termine di accoppiamento $M_{12}\ddot\alpha$ è trattato come disturbo/compensazione);
2. corrente: $i = \tau_1 / k_t$;
3. tensione: $V = R\,i + k_e\,\dot\theta$ con $k_e = k_t$ in unità SI.

Manca la **resistenza $R$** in `parameters.hpp`: senza di essa il percorso non è chiudibile in unità coerenti, ma è quello fisicamente coerente con `ActuatorLimits` e con un driver SimpleFOC in controllo di corrente.

**Percorso B — voltage path (alternativa).** Tensione proporzionale al comando, saturata a 24 V, con compensazione della back-EMF tramite $k_V$:

$$V = \operatorname{clamp}\!\left(K_u\,\ddot\theta_{cmd} + k_V\,\dot\theta,\ -24,\ +24\right)$$

più grezzo, ma non richiede $R$.

### 8.1 Il modello `Motor` oggi

La classe `Motor` (`motor.hpp`) deriva $k_t$ e $k_V$ con due modalità selezionabili a runtime (`setMode`), con `RTRPS` $= 2\pi/60$:

| Modalità | $k_t$ | $k_V$ |
|---|---|---|
| `mode == 0` (default) | $\left(\dfrac{P_{nom}}{n_{RPM}\cdot RTRPS}\right)\dfrac{1}{3.30}$ | $1/(k_t\cdot RTRPS)$ |
| `mode == 1` | $1/(k_V\cdot RTRPS)$ | $RPM_{max}/V_{dc}$ |

Con i valori attuali si ottiene $k_t \approx 0.048$ (mode 0) oppure $k_t \approx 0.060$, $k_V \approx 160\ \mathrm{RPM/V}$ (mode 1). **Da rivedere**: la costante $3.30$, la coerenza dimensionale tra le due modalità e l'uso di `poles` sono da documentare/verificare; `maxTorque` e `maxCurrent` sono vuoti, quindi oggi non esiste un limite di coppia esplicito.

---

## 9. Architettura software

### 9.1 Struttura del repository

```
pendulum/
├── furuta/
│   ├── CMakeLists.txt              # C++17, target "furuta"
│   ├── include/furuta/
│   │   ├── parameters.hpp          # geometria, inerzie, limiti attuatore
│   │   ├── sensor_data.hpp         # SensorData: theta, alpha + velocità filtrate
│   │   ├── velocity_filters.hpp    # AS5600VelocityEstimator (LowPass/Kalman/AlphaBeta)
│   │   ├── math_utils.hpp          # rpm→rad/s, clamp, homing, referenceAngle
│   │   ├── energy_swing_up.hpp     # E, E*, errE (legge di controllo: TODO)
│   │   └── motor.hpp               # stima di kt e kV
│   ├── src/main.cpp                # smoke test su host con sensori simulati
│   ├── data/
│   │   ├── data_extraction.py      # logger seriale → CSV
│   │   └── data.csv                # log (attualmente vuoto)
│   └── tests/                      # vuoto
└── README.md
```

### 9.2 Flusso dati previsto

```mermaid
flowchart LR
    subgraph SENSORI["Sensori (da collegare sulla board)"]
        A1["AS5600 braccio"] -.->|I2C| SD
        A2["AS5600 pendolo"] -.->|I2C| SD
    end
    SD["SensorData<br/>theta, alpha"]
    SD --> EST["AS5600VelocityEstimator x2<br/>thetaDotFiltered, alphaDotFiltered"]
    EST --> CTRL["EnergySwingUpController<br/>E, E*, errE"]
    CTRL --> LAW["Legge swing-up + saturazione"]
    LAW --> MAP["Mappatura theta_ddot → tau → V"]
    MAP --> MOT["Motor: kt, kV"]
    MOT --> SF["SimpleFOC"] -.-> BLDC["Motore BLDC"]
    EST -.-> LOG["log seriale"]
    LOG -.-> CSV["data_extraction.py → data.csv"]

    classDef impl fill:#d4f7d4,stroke:#2e7d32;
    classDef todo fill:#fff3cd,stroke:#b8860b,stroke-dasharray: 4 3;
    class SD,EST,CTRL impl;
    class LAW,MAP,MOT,SF,BLDC,A1,A2,LOG,CSV todo;
```

In verde ciò che è implementato, in giallo tratteggiato ciò che è da implementare/collegare. `main.cpp` oggi è un **smoke test**: genera angoli simulati e stampa `mode`, `kt`, `kV` dopo un `setMode(1)`; i filtri e il controllore non sono ancora cablati nel loop.

### 9.3 Utilità di homing e quiescenza

In `math_utils.hpp`:

- `atRest(angle, angleDot)`: vero se l'asse è fermo sul riferimento, con tolleranza sull'angolo $2\ \mathrm{LSB} \approx 3.1\ \mathrm{mrad}$ e sulla velocità $0.4\ \mathrm{rad/s}$. Serve anche il criterio di quiete, perché l'angolo passa per lo zero anche a velocità massima; per un homing robusto il commento raccomanda più campioni consecutivi;
- `referenceAngle(rawAngle, offset)`: riporta l'angolo nel range $[-2\pi, 2\pi]$;
- `homeTarget(sensorDirection, offset)`: tiene conto del verso di montaggio del sensore.

### 9.4 Acquisizione dati

`data/data_extraction.py` legge da `/dev/ttyACM0` a 115200 baud, antepone un timestamp ISO e appende le righe (split su virgola) a `data/data.csv`. Nota: nel codice `f.flush` è scritto senza parentesi (non esegue il flush); i dati restano comunque corretti perché il file viene chiuso alla fine. Il file CSV attuale è vuoto: la pipeline non è ancora stata usata con dati reali.

---

## 10. Build ed esecuzione

Requisiti: CMake $\ge 3.16$, compilatore C++17.

```bash
cmake -S . -B build
cmake --build build
./build/furuta
```

`main.cpp` stampa i valori di `kt` e `kV` nelle due modalità del motore. Per il logger seriale (richiede `pyserial`, il venv del repository è già presente):

```bash
python data/data_extraction.py
```

Lo script ha percorso porta e file di output **hardcoded** (`/dev/ttyACM0`, percorso assoluto di `data.csv`): vanno adattati alla macchina.

---

## 11. Stato dell'implementazione e roadmap

### 11.1 Stato attuale

| Componente | File | Stato | Note |
|---|---|---|---|
| Parametri e limiti | `parameters.hpp` | ✅ presente | `mu`, `J_xx`, `poles` non usati; `maxTorque`/`maxCurrent` vuoti |
| Lettura angoli | `sensor_data.hpp`, `main.cpp` | ⚠️ solo mock host | driver I2C AS5600 da scrivere |
| Stima velocità | `velocity_filters.hpp` | ✅ implementata | non istanziata in `main.cpp` |
| Homing / quiete | `math_utils.hpp` | ✅ implementata | non usata |
| Energia pendolo ($E$, $E^\star$, `errE`) | `energy_swing_up.hpp` | ✅ implementata | — |
| Legge di swing-up + saturazione | — | ❌ da implementare | `controlOutput` è uno stub con campo `velocity` |
| Modello dinamico $M, h, G$ | — | ❌ da implementare | mancano $\hat J_0$ (cioè $J_1$), $b_1, b_2$ |
| RK4 + `stateDerivative` | — | ❌ da implementare | progetto: $h = 1$ ms, ZOH |
| Mappatura $\ddot\theta \to \tau \to V$ | — | ❌ aperta | mancano $R$, $i_{max}$, $k_t$ coerenti |
| Log CSV | `data/data_extraction.py` | ✅ presente | `data.csv` vuoto; flush senza parentesi |
| Test | `tests/` | ❌ vuoto | — |
| Firmware board (SimpleFOC + AS5600) | — | ❌ da fare | oggi solo host; `main.ino` rimosso in passato |

### 11.2 Roadmap

1. **`dynamics.hpp`**: $M(q)$, $h(q,\dot q)$, $G(q)$; completare $\hat J_0$ (misura di $J_1$ o identificazione complessiva). Verifica: energia meccanica costante nel tempo a coppia nulla e attriti nulli.
2. **`integrator.hpp`**: RK4 + `stateDerivative`. Verifica: pendolo libero ($\dot\theta = 0$) con confronto al piccolo angolo e drift di energia $\ll 1\%$ su 10 s con $h = 1$ ms.
3. **Legge di swing-up + saturazione** in simulazione chiusa. Verifica: da $\alpha = 0$ a $E \approx E^\star$ entro un tempo prefissato, senza saturare inutilmente.
4. **Identificazione**: $b_1, b_2$, $R$, $i_{max}$; revisione di $k_t, k_V$ e della costante $3.30$. Poi mappatura a tensione.
5. **Port su board**: SimpleFOC + due AS5600 su I2C, homing, verifica della frequenza reale del loop rispetto a $h$.
6. **Stabilizzazione**: controllore in cima (LQR o PID con scheduling) e logica di commutazione swing-up ↔ stabilize.
7. **Test e analisi**: riempire `tests/`, pulire il logger e costruire l'analisi dei CSV.

---

## 12. Riferimenti

- K. Furuta, M. Yamakita, S. Kobayashi, *Swing-up control of inverted pendulum using pseudo-state feedback*, Journal of Systems and Control Engineering, 206(6), 1992.
- B. S. Cazzolato, Z. Prime, *On the Dynamics of the Furuta Pendulum*, Journal of Control Science and Engineering, 2011, Article ID 528341 — [PDF](https://downloads.hindawi.com/journals/jcse/2011/528341.pdf).
- K. J. Åström, K. Furuta, *Swinging up a pendulum by energy control*, Automatica, 36(2):287–295, 2000.
- J. Á. Acosta, *Furuta's Pendulum: A Conservative Nonlinear Model for Theory and Practise*, Mathematical Problems in Engineering, 2010, Article ID 742894.
- [Wikipedia — Furuta pendulum](https://en.wikipedia.org/wiki/Furuta_pendulum) (equazioni di Cazzolato & Prime, geometria di riferimento).
- [SimpleFOC](https://simplefoc.com/) (driver motore previsto) e datasheet AS5600 (encoder magnetico 12 bit).
