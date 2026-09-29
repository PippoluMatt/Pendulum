#ifndef VELOCITY_FILTERS_HPP
#define VELOCITY_FILTERS_HPP

#include <cmath>

namespace furuta
{
    enum class VelocityFilterMode { LowPass, Kalman, AlphaBeta };

    // Stima della velocita' di un AS5600 (encoder magnetico assoluto a 12 bit,
    // 4096 conteggi/giro).
    //
    // Tre modalita', scelte a runtime con l'enum:
    // - LowPass: derivata grezza delta/dt filtrata con EMA del primo ordine.
    //   La lettura e' quantizzata (1 LSB = 2*pi/4096 rad), la derivata grezza
    //   amplifica il rumore di 1/dt; l'EMA ha guadagno unitario in continua,
    //   quindi una velocita' costante non viene attenuata.
    // - Kalman: modello a velocita' costante, stato [angolo, velocita'].
    //   La velocita' e' uno stato, non una derivata; R si ricava dalla
    //   quantizzazione, qTheta/qOmega sono rumore di processo da tarare.
    // - AlphaBeta: stesso modello del Kalman con guadagni fissi
    //   (angle += alpha * innovazione, velocity += beta/dt * innovazione).
    //   Tuning: 0 < alpha < 2, 0 < beta < 4 - 2*alpha; con
    //   beta = alpha^2 / (2 - alpha) la risposta e' criticamente smorzata.
    //
    // Perche' il wrap: l'AS5600 e' assoluto su un solo giro, agli estremi la
    // lettura salta da ~2*pi a 0 (serve per alpha, il pendolo puo' ruotare).
    //
    // Un'istanza per sensore: una per il braccio, una per il pendolo.
    class AS5600VelocityEstimator
    {
    public:
        // mode: filtro attivo
        // fc_hz: frequenza di taglio a -3 dB [Hz] (solo LowPass)
        // fs_hz: frequenza di campionamento del loop [Hz]
        // qTheta, qOmega: rumore di processo (solo Kalman)
        // alpha, beta: guadagni (solo AlphaBeta)
        // restHold_s: se l'angolo resta entro 2 LSB per questo tempo la
        //   velocita' e' forzata a zero (0 = disabilitato).
        // restZone_rad: semilarghezza attorno ad alpha = 0 in cui il lock e'
        //   attivo: a 15 gradi si applica solo al pendolo appeso, mai a
        //   SWING/BALANCE, dove alpha e' vicino a pi e serve la banda piena.
        AS5600VelocityEstimator(VelocityFilterMode mode,
                                float fc_hz,
                                float fs_hz,
                                float qTheta = 1.0e-6f,
                                float qOmega = 1.0e-2f,
                                float alpha = 0.5f,
                                float beta = 0.1667f,
                                float restHold_s = 0.0f,
                                float restZone_rad = static_cast<float>(M_PI)) noexcept
            : mode_{mode}, invDt_{fs_hz}, dt_{1.0f / fs_hz},
              alpha_{alpha}, beta_{beta}, restHold_{restHold_s},
              restZone_{restZone_rad}
        {
            rc_ = 1.0f / (2.0f * static_cast<float>(M_PI) * fc_hz);
            betaLp_ = dt_ / (rc_ + dt_);

            // Rumore di quantizzazione: varianza uniforme su 1 LSB.
            constexpr float lsb = 2.0f * static_cast<float>(M_PI) / 4096.0f;
            r_ = lsb * lsb / 12.0f;

            q_[0][0] = qTheta;
            q_[1][1] = qOmega;
        }

        // Aggancia lo stato alla posizione attuale senza generare uno spike
        // (es. dopo l'homing o un reset del loop).
        void reset(double angle_rad) noexcept
        {
            prevAngle_ = angle_rad;
            velocity_ = 0.0f;

            kAngle_ = angle_rad;
            kVelocity_ = 0.0f;
            p_[0][0] = r_;
            p_[0][1] = 0.0f;
            p_[1][0] = 0.0f;
            p_[1][1] = 100.0f;  // velocita' iniziale sconosciuta

            abAngle_ = angle_rad;
            abVelocity_ = 0.0f;

            restAnchor_ = angle_rad;
            restTime_ = 0.0f;
            restLock_ = false;

            initialized_ = true;
        }

        // Nuova lettura [rad] -> velocita' filtrata [rad/s].
        // Il primo campione dopo un reset serve solo a inizializzare lo stato
        // e restituisce 0.
        float update(double angle_rad) noexcept
        {
            if (!initialized_)
            {
                reset(angle_rad);
                return 0.0f;
            }

            // A riposo l'uscita deve essere zero, non il dither di
            // quantizzazione: nella zona di riposo, angolo fermo entro 2 LSB
            // per restHold_ blocca a zero; un movimento vero sblocca.
            const double restAngle = wrapPi(angle_rad);
            if (restHold_ > 0.0f && std::abs(restAngle) <= restZone_)
            {
                if (std::abs(wrapPi(angle_rad - restAnchor_)) > restWindow_)
                {
                    restAnchor_ = angle_rad;
                    restTime_ = 0.0f;
                    restLock_ = false;
                }
                else if (!restLock_)
                {
                    restTime_ += dt_;
                    if (restTime_ >= restHold_) restLock_ = true;
                }
                if (restLock_)
                {
                    // Allinea gli stati d'angolo e azzera la velocita': allo
                    // sblocco l'innovazione riparte dall'ultimo campione.
                    prevAngle_ = angle_rad;
                    kAngle_ = angle_rad;
                    kVelocity_ = 0.0f;
                    abAngle_ = angle_rad;
                    abVelocity_ = 0.0f;
                    velocity_ = 0.0f;
                    return 0.0f;
                }
            }
            else
            {
                restLock_ = false;
            }

            switch (mode_)
            {
                case VelocityFilterMode::LowPass:   return updateLowPass(angle_rad);
                case VelocityFilterMode::Kalman:    return updateKalman(angle_rad);
                case VelocityFilterMode::AlphaBeta: return updateAlphaBeta(angle_rad);
            }
            return 0.0f;
        }

        // Per loop non periodici: intervallo reale fra due letture, in secondi
        // (strettamente positivo). Senza questo dato fs_hz resta un'ipotesi.
        float update(double angle_rad, float dt_seconds) noexcept
        {
            dt_ = dt_seconds;
            invDt_ = 1.0f / dt_;
            betaLp_ = dt_ / (rc_ + dt_);
            return update(angle_rad);
        }

    private:
        // L'angolo predetto puo' accumulare piu' giri: una sola correzione di
        // 2*pi non basta. L'innovazione deve restare in [-pi, pi].
        static double wrapPi(double delta) noexcept
        {
            return std::remainder(delta, 2.0 * M_PI);
        }

        float updateLowPass(double angle_rad) noexcept
        {
            const double delta = wrapPi(angle_rad - prevAngle_);
            prevAngle_ = angle_rad;

            const float velocityRaw = static_cast<float>(delta) * invDt_;
            // Forma incrementale: piu' stabile di beta*v + (1-beta)*v_prev
            // quando beta e' piccolo (nessuna cancellazione catastrofica).
            velocity_ += betaLp_ * (velocityRaw - velocity_);
            return velocity_;
        }

        float updateKalman(double angle_rad) noexcept
        {
            // Predict: angle += v*dt, P = F*P*F' + Q
            const double predictedAngle = kAngle_ + kVelocity_ * dt_;
            const float p00 = p_[0][0] + dt_ * (p_[0][1] + p_[1][0])
                            + dt_ * dt_ * p_[1][1] + q_[0][0];
            const float p01 = p_[0][1] + dt_ * p_[1][1];
            const float p10 = p_[1][0] + dt_ * p_[1][1];
            const float p11 = p_[1][1] + q_[1][1];

            // Update: H = [1 0], innovazione wrappata.
            const double innovation = wrapPi(angle_rad - predictedAngle);
            const float s  = p00 + r_;
            const float k0 = p00 / s;
            const float k1 = p10 / s;

            kAngle_    = predictedAngle + k0 * innovation;
            kVelocity_ += k1 * innovation;

            p_[0][0] = (1.0f - k0) * p00;
            p_[0][1] = (1.0f - k0) * p01;
            p_[1][0] = p10 - k1 * p00;
            p_[1][1] = p11 - k1 * p01;

            prevAngle_ = angle_rad;
            return kVelocity_;
        }

        float updateAlphaBeta(double angle_rad) noexcept
        {
            const double predictedAngle = abAngle_ + abVelocity_ * dt_;
            const double innovation = wrapPi(angle_rad - predictedAngle);

            abAngle_ = predictedAngle + alpha_ * innovation;
            abVelocity_ += (beta_ * invDt_) * static_cast<float>(innovation);

            prevAngle_ = angle_rad;
            return abVelocity_;
        }

        VelocityFilterMode mode_;
        float invDt_;
        float dt_;

        // LowPass
        float rc_{0.0f};
        float betaLp_{0.0f};
        float velocity_{0.0f};

        // Kalman
        float q_[2][2]{};
        float r_{0.0f};
        double kAngle_{0.0};
        float kVelocity_{0.0f};
        float p_[2][2]{};

        // AlphaBeta
        const float alpha_;
        const float beta_;
        double abAngle_{0.0};
        float abVelocity_{0.0f};

        // Rest-lock
        const float restHold_{0.0f};
        const float restZone_{static_cast<float>(M_PI)};
        float restAnchor_{0.0f};
        float restTime_{0.0f};
        bool restLock_{false};
        static constexpr float restWindow_{2.0f * 2.0f * static_cast<float>(M_PI) / 4096.0f};

        double prevAngle_{0.0};
        bool initialized_{false};
    };
} // furuta namespace

#endif
