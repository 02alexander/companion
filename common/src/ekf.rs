use nalgebra::SMatrix;

use crate::model::{INERTIA_RATIO, RADIUS, WHEEL_STATIC_GAIN, WHEEL_TIME_CONSTANT};

pub type Mat<const R: usize, const C: usize> = SMatrix<f32, R, C>;

pub trait Model<const NX: usize, const NY: usize> {
    fn f(&self, x: Mat<NX, 1>, u: f32) -> Mat<NX, 1>;
    fn fprim(&self, x: Mat<NX, 1>) -> Mat<NX, NX>;

    #[allow(non_snake_case)]
    fn Q(&self) -> Mat<NX, NX>;

    fn h(&self, x: Mat<NX, 1>) -> Mat<NY, 1>;
    fn hprim(&self, x: Mat<NX, 1>) -> Mat<NY, NX>;

    #[allow(non_snake_case)]
    fn R(&self) -> Mat<NY, NY>;
}

#[allow(non_snake_case)]
pub struct LinearModel<const NX: usize, const NY: usize> {
    pub A: Mat<NX, NX>,
    pub B: Mat<NX, 1>,
    pub C: Mat<NY, NX>,
    pub D: Mat<NY, 1>,
    pub Q: Mat<NX, NX>,
    pub R: Mat<NY, NY>,
}

impl<const R: usize, const C: usize> Model<R, C> for LinearModel<R, C> {
    fn Q(&self) -> Mat<R, R> {
        self.Q
    }

    fn f(&self, x: Mat<R, 1>, u: f32) -> Mat<R, 1> {
        self.A * x + self.B * u
    }

    fn fprim(&self, _x: Mat<R, 1>) -> Mat<R, R> {
        self.A
    }

    fn h(&self, x: Mat<R, 1>) -> Mat<C, 1> {
        self.C * x
    }

    fn hprim(&self, _x: Mat<R, 1>) -> Mat<C, R> {
        self.C
    }

    fn R(&self) -> Mat<C, C> {
        self.R
    }
}

#[allow(non_snake_case)]
pub struct NLModel {
    pub dt: f32,
}

impl Model<3, 1> for NLModel {
    fn Q(&self) -> Mat<3, 3> {
        Mat::from_rows(&[
            [0.00001, 0.0, 0.0].into(),
            [0.0, 0.0, 0.0].into(),
            [0.0, 0.0, 100000000.0 * 100000000.0].into(),
        ])
    }

    fn f(&self, x: Mat<3, 1>, u: f32) -> Mat<3, 1> {
        Mat::<3, 1>::from_column_slice(&[
            x[0] - self.dt * x[0] / WHEEL_TIME_CONSTANT,
            x[1] + self.dt * x[2],
            x[2] + self.dt
                * (INERTIA_RATIO * x[0] / WHEEL_TIME_CONSTANT + 9.81 * libm::sinf(x[1]) / RADIUS
                    - 0.2 * x[2]),
        ]) + Mat::<3, 1>::from_column_slice(&[
            self.dt * WHEEL_STATIC_GAIN / WHEEL_TIME_CONSTANT,
            0.0,
            -self.dt * INERTIA_RATIO * WHEEL_STATIC_GAIN / WHEEL_TIME_CONSTANT,
        ]) * u
    }

    fn fprim(&self, x: Mat<3, 1>) -> Mat<3, 3> {
        Mat::from_rows(&[
            [1.0 - self.dt / WHEEL_TIME_CONSTANT, 0.0, 0.0].into(),
            [0.0, 1.0, self.dt].into(),
            [
                -self.dt * INERTIA_RATIO / WHEEL_TIME_CONSTANT,
                9.81 * libm::cosf(x[1]) / RADIUS,
                1.0 - 0.2,
            ]
            .into(),
        ])
    }

    fn h(&self, x: Mat<3, 1>) -> Mat<1, 1> {
        [x[1]].into()
    }

    fn hprim(&self, _x: Mat<3, 1>) -> Mat<1, 3> {
        [0.0, 1.0, 0.0].into()
    }

    fn R(&self) -> Mat<1, 1> {
        let std = 0.0001;
        Mat::identity() * std * std
    }
}

#[allow(non_snake_case)]
pub struct EKF<const NX: usize, const NY: usize, M> {
    pub x: Mat<NX, 1>,
    pub P: Mat<NX, NX>,
    pub model: M,
}

impl<const NX: usize, const NY: usize, M> EKF<NX, NY, M>
where
    M: Model<NX, NY>,
{
    pub fn from_model(model: M) -> Self {
        EKF {
            x: Mat::zeros(),
            P: 10000.0 * Mat::identity(),
            model,
        }
    }
    pub fn time_update(&mut self, u: f32) {
        self.x = self.model.f(self.x, u);
        let fprim = self.model.fprim(self.x);
        self.P = self.model.Q() + fprim * self.P * fprim.transpose()
    }

    pub fn measurment_update(&mut self, meas: Mat<NY, 1>) -> Option<()> {
        let error = meas - self.model.h(self.x);
        self.measurment_update_from_error(error)
    }

    /// Performs measurement update a specified error. where error = y - yhat
    pub fn measurment_update_from_error(&mut self, error: Mat<NY, 1>) -> Option<()> {
        let hprim = self.model.hprim(self.x);
        let s = self.model.R() + hprim * self.P * hprim.transpose();
        let inv_s = s.try_inverse()?;

        let k = self.P * hprim.transpose() * inv_s;
        self.P = self.P - self.P * hprim.transpose() * inv_s * hprim * self.P;
        self.x += k * error;
        Some(())
    }
}
