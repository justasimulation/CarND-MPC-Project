#ifndef MPC_INDICES_H
#define MPC_INDICES_H

//Contains indices of values that are passed to optimizer in a flat array
//Indices depend on number of values of each value type N.
//E.g. if N = 3, then the array will contain the following values
//[x0, x1, x2, y0, y1, y2, psi0, psi1, psi2...]
struct Indices
{
    public:
        Indices(size_t N)
        {
            this->N = N;
            this->x_start = 0;
            this->y_start = x_start + N;
            this->psi_start = y_start + N;
            this->v_start = psi_start + N;
            this->cte_start = v_start + N;
            this->epsi_start = cte_start + N;
            this->delta_start = epsi_start + N;
            this->a_start = delta_start + N - 1;
        };

        size_t N;
        size_t x_start;
        size_t y_start;
        size_t psi_start;
        size_t v_start;
        size_t cte_start;
        size_t epsi_start;
        size_t delta_start;
        size_t a_start;
};

#endif //MPC_INDICES_H
