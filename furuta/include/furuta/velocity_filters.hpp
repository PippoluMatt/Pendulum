#include <cmath>

namespace furuta
{

    // Stima della velocita' di un AS5600 (encoder magnetico assoluto a 12 bit,
    // 4096 conteggi/giro) per differenze finite filtrate.
    //
    // Perche' il filtro: la lettura e' quantizzata (1 LSB = 2*pi/4096 rad),
    // quindi la derivata grezza delta/dt amplifica il rumore di quantizzazione
    // di 1/dt. Il filtro IIR del primo ordine (EMA)
    //     v[k] = v[k-1] + beta * (vRaw[k] - v[k-1])
    // smorza il rumore con guadagno unitario in continua: una velocita'
    // costante non viene attenuata.
    //
    // Perche' il wrap: l'AS5600 e' assoluto su un solo giro, agli estremi la
    // lettura salta da ~2*pi a 0 (serve per alpha, il pendolo puo' ruotare).
    //
    // Un'istanza per sensore: una per il braccio, una per il pendolo.
    enum class VelocityFilterMode {LowPass, Kalman};
    class AS5600VelocityEstimator
    {
    public:
        // fc_hz: frequenza di taglio a -3 dB [Hz]
        // fs_hz: frequenza di campionamento del loop [Hz]
        AS5600VelocityEstimator(float fc_hz, float fs_hz) noexcept
            : invDt_{fs_hz}
        {
            const float dt = 1.0f / fs_hz;
            const float rc = 1.0f / (2.0f * static_cast<float>(M_PI) * fc_hz);
            beta_ = dt / (rc + dt);
        }

        // Aggancia lo stato alla posizione attuale senza generare uno spike
        // (es. dopo l'homing o un reset del loop).
        void reset(double angle_rad) noexcept
        {
            prevAngle_ = angle_rad;
            velocity_ = 0.0f;
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

            double delta = angle_rad - prevAngle_;
            prevAngle_ = angle_rad;

            // AS5600 assoluto su un giro: riporta il salto di lettura in (-pi, pi]
            if (delta > M_PI)          delta -= 2.0 * M_PI;
            else if (delta < -M_PI)    delta += 2.0 * M_PI;

            const float velocityRaw = static_cast<float>(delta) * invDt_;
            // Forma incrementale: piu' stabile di beta*v + (1-beta)*v_prev
            // quando beta e' piccolo (nessuna cancellazione catastrofica).
            velocity_ += beta_ * (velocityRaw - velocity_);
            return velocity_;
        }

    private:
        const float invDt_;
        float beta_{0.0f};
        double prevAngle_{0.0};
        float velocity_{0.0f};
        bool initialized_{false};
    };
} // furuta namespace