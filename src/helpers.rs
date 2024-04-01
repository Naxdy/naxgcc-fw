use core::ops::{Add, AddAssign, Deref, Sub, SubAssign};
use defmt::Format;
use packed_struct::PackedStruct;

/// wrapper type because packed_struct doesn't implement float
/// packing by default
#[derive(Debug, Format, Clone, Default, Copy)]
pub struct PackedFloat(pub f32);

impl Add for PackedFloat {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self(self.0 + rhs.0)
    }
}

impl Add<f32> for PackedFloat {
    type Output = Self;

    fn add(self, rhs: f32) -> Self::Output {
        Self(self.0 + rhs)
    }
}

impl Sub for PackedFloat {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self(self.0 - rhs.0)
    }
}

impl Sub<f32> for PackedFloat {
    type Output = Self;

    fn sub(self, rhs: f32) -> Self::Output {
        Self(self.0 - rhs)
    }
}

impl SubAssign for PackedFloat {
    fn sub_assign(&mut self, rhs: Self) {
        self.0 -= rhs.0;
    }
}

impl SubAssign<f32> for PackedFloat {
    fn sub_assign(&mut self, rhs: f32) {
        self.0 -= rhs;
    }
}

impl AddAssign for PackedFloat {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl AddAssign<f32> for PackedFloat {
    fn add_assign(&mut self, rhs: f32) {
        self.0 += rhs;
    }
}

pub trait ToRegularArray<const T: usize> {
    fn to_regular_array(&self) -> &[f32; T];
}

impl<const T: usize> ToRegularArray<T> for [PackedFloat; T] {
    fn to_regular_array(&self) -> &[f32; T] {
        unsafe { &*(self as *const _ as *const _) }
    }
}

pub trait ToPackedFloatArray<const T: usize> {
    fn to_packed_float_array(&self) -> &[PackedFloat; T];
}

impl<const T: usize> ToPackedFloatArray<T> for [f32; T] {
    fn to_packed_float_array(&self) -> &[PackedFloat; T] {
        unsafe { &*(self as *const _ as *const _) }
    }
}

impl PackedStruct for PackedFloat {
    type ByteArray = [u8; 4];

    fn pack(&self) -> packed_struct::PackingResult<Self::ByteArray> {
        Ok(self.to_be_bytes())
    }

    fn unpack(src: &Self::ByteArray) -> packed_struct::PackingResult<Self> {
        Ok(Self(f32::from_be_bytes(*src)))
    }
}

impl Deref for PackedFloat {
    type Target = f32;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl From<f32> for PackedFloat {
    fn from(f: f32) -> Self {
        Self(f)
    }
}

#[derive(Debug, Clone, Format, Default, Copy)]
pub struct XyValuePair<T> {
    pub x: T,
    pub y: T,
}
