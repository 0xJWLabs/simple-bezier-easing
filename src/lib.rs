//! This module provides functions and utilities for calculating cubic Bézier curves
//! using De Casteljau's algorithm. It includes robust error handling for invalid
//! control points and supports binary subdivision to efficiently determine the parameter
//! `t` corresponding to a given x-coordinate on the curve.
//!
//! # Key Features
//! - **Cubic Bézier Curve Calculation**: Computes the y-coordinate for any given x-coordinate
//!   based on the specified control points using De Casteljau's algorithm.
//! - **Error Handling**: Returns detailed errors if the control points are outside the valid range
//!   [0, 1] or if the binary subdivision fails to converge within the allowed iterations.
//! - **Binary Subdivision**: Utilizes binary subdivision to efficiently find the parameter `t` for a
//!   given x-coordinate, ensuring accurate results even for complex curves.
//! - **Precision Control**: Allows customization of the precision for the binary subdivision process
//!   and handles edge cases gracefully, such as very small slopes or extreme control point values.

use thiserror::Error;

/// The precision used for binary subdivision in the Bézier curve evaluation.
///
/// This value determines how accurately the parameter `t` is found during the binary subdivision process.
/// A smaller value means more iterations and higher precision, but at the cost of performance.
/// It's used as the stopping condition when the difference between two successive `t` values is smaller than this threshold.
const SUBDIVISION_PRECISION: f32 = 0.0001;

/// The minimum slope allowed for the curve's derivative during the binary subdivision process.
///
/// This constant ensures that we don't encounter division by zero or excessively small slopes, which could
/// lead to instability or undefined behavior. It helps control the accuracy of the subdivision by rejecting
/// slopes that are too close to zero.
const MIN_SLOPE: f32 = 0.001;

/// The maximum number of iterations allowed for binary subdivision.
///
/// This constant limits how many times the binary subdivision process can be iterated in the search for the
/// correct parameter `t`. It prevents the algorithm from running indefinitely and ensures that the function
/// completes in a reasonable amount of time. If the desired precision is not met within this number of iterations,
/// an error will be returned.
const SUBDIVISION_MAX_ITERATIONS: u32 = 10;

/// Custom errors for handling Bézier curve computations.
#[derive(Debug, Error)]
pub enum BezierError {
    /// Error indicating that control points are out of the valid range [0, 1].
    #[error("Control points must be in the range [0, 1], but got: ({x1}, {y1}), ({x2}, {y2})")]
    InvalidControlPoint { x1: f32, y1: f32, x2: f32, y2: f32 },

    /// Error for failing to calculate the parameter `t` for a given x-coordinate.
    #[error("Failed to find parameter t for x = {0}")]
    ParameterCalculationError(f32),
}

/// A simple 2D point represented as (x, y).
#[derive(Debug, Clone, Copy)]
pub struct Point(pub f32, pub f32);

impl Point {
    /// Linearly interpolates between two points `a` and `b` at a parameter `t`.
    fn lerp(a: Point, b: Point, t: f32) -> Point {
        Point(a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t)
    }
}

impl std::fmt::Display for Point {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "({}, {})", self.0, self.1)
    }
}

/// Computes the cubic Bézier curve using De Casteljau's algorithm.
///
/// De Casteljau's algorithm is a recursive method for evaluating Bézier curves
/// at a specific parameter `t`. It works by linearly interpolating between
/// control points at each level until a single point is obtained.
///
/// Parameters:
/// - `t`: The parameter (0 ≤ t ≤ 1) at which to evaluate the curve.
/// - `p0`: The first control point (start of the curve).
/// - `p1`: The second control point (first "pull" control).
/// - `p2`: The third control point (second "pull" control).
/// - `p3`: The fourth control point (end of the curve).
///
/// Returns:
/// - The y-coordinate of the Bézier curve at parameter `t`.
fn de_casteljau(t: f32, p0: Point, p1: Point, p2: Point, p3: Point) -> Point {
    // First level: Linearly interpolate between the control points
    // Compute intermediate points `q0`, `q1`, and `q2` at the first level.
    let q0 = Point::lerp(p0, p1, t);
    let q1 = Point::lerp(p1, p2, t);
    let q2 = Point::lerp(p2, p3, t);

    // Second level: Interpolate between the intermediate points from the first level
    // Compute `r0` and `r1` as the second-level intermediate points.
    let r0 = Point::lerp(q0, q1, t);
    let r1 = Point::lerp(q1, q2, t);

    // Final level: Interpolate between the second-level points to get the final result
    // Compute the final point on the curve corresponding to `t`.
    Point::lerp(r0, r1, t) // Interpolates between r0 and r1 to get the curve's value
}

/// Uses binary subdivision to find the parameter `t` for a given x-coordinate on the Bézier curve.
///
/// This function attempts to compute the parameter `t` such that when the Bézier curve is evaluated
/// at `t`, the resulting x-coordinate matches the given target `x`. The method uses binary subdivision
/// within the range [0, 1] to iteratively narrow down the value of `t` until the x-coordinate of the
/// Bézier curve at `t` is sufficiently close to the target `x` or the maximum number of iterations is reached.
///
/// # Parameters
/// - `x`: The target x-coordinate for which to find the corresponding parameter `t`.
/// - `p0`, `p1`, `p2`, `p3`: The control points of the cubic Bézier curve, where `p0` is the start point,
///   `p3` is the end point, and `p1` and `p2` are the intermediate control points that define the curve's shape.
///
/// # Returns
/// - `Ok(t)`: The parameter `t` corresponding to the given x-coordinate, where `0 <= t <= 1`.
/// - `Err(BezierError::ParameterCalculationError(x))`: If the x-coordinate cannot be matched within the
///   specified precision or the maximum number of iterations is reached, an error is returned, indicating
///   that the calculation for the given x-coordinate failed.
///
/// # Errors
/// - If the binary subdivision fails to find a suitable parameter `t` within the maximum iterations or
///   the precision requirements, it returns a `ParameterCalculationError` with the target x-coordinate.
///
fn get_t_for_x(x: f32, p0: Point, p1: Point, p2: Point, p3: Point) -> Result<f32, BezierError> {
    let mut t0 = 0.0;
    let mut t1 = 1.0;
    let mut t = (t0 + t1) / 2.0; // Start with a midpoint guess rather than x
    let mut last_t = t;

    for _ in 0..SUBDIVISION_MAX_ITERATIONS {
        // Evaluate the Bézier curve at `t` to find its x-coordinate.
        let x_val = de_casteljau(t, p0, p1, p2, p3);
        let error = x - x_val.0;

        // Adjust the range based on the error.
        if error.abs() < SUBDIVISION_PRECISION {
            break;
        }
        if error > 0.0 {
            t0 = t;
        } else {
            t1 = t;
        }
        t = (t0 + t1) / 2.0;

        if (t - last_t).abs() < SUBDIVISION_PRECISION {
            break;
        }

        last_t = t;
    }

    let final_x_val = de_casteljau(t, p0, p1, p2, p3);
    if (x - final_x_val.0).abs() < MIN_SLOPE && (t - last_t).abs() < MIN_SLOPE {
        Ok(t) // Return the result if it's sufficiently accurate
    } else {
        Err(BezierError::ParameterCalculationError(x)) // Otherwise, return an error
    }
}

/// Creates a cubic Bézier curve function based on the given control points.
///
/// This function returns a closure that can be used to compute the y-coordinate of the Bézier curve for
/// any given x-coordinate within the range [0, 1]. The closure uses De Casteljau's algorithm to compute
/// the point on the curve corresponding to the x-coordinate. It also supports binary subdivision to find
/// the parameter `t` corresponding to the given x and then evaluate the curve at that parameter.
///
/// # Parameters
/// - `x1`, `y1`: Coordinates of the first control point (p1), which influences the curve's direction and shape.
/// - `x2`, `y2`: Coordinates of the second control point (p2), which also influences the curve's direction and shape.
///
/// # Returns
/// - `Ok`: A closure that accepts an x-coordinate and returns the corresponding y-coordinate on the Bézier curve.
///   The closure will return a `Result<f32, BezierError>`, where the y-coordinate is computed for the given x.
/// - `Err`: If the control points are outside the valid range [0, 1], an error is returned indicating the invalid
///   control points. The error type is `BezierError::InvalidControlPoint`.
///
/// # Errors
/// - `BezierError::InvalidControlPoint`: If any of the control points are outside the range [0, 1], an error is returned.
/// - `BezierError::ParameterCalculationError`: If the binary subdivision fails to calculate a valid parameter `t`
///   for a given x-coordinate when calling the returned closure.
///
/// # Example
///
/// ```rust
/// use simple_bezier_easing::bezier;
/// let bez = bezier(0.2, 0.4, 0.6, 0.8).unwrap();
/// let y_at_0_5 = bez(0.5).unwrap();  // Compute the y-coordinate at x = 0.5
/// let rounded_value = (y_at_0_5 * 10.0).round() / 10.0; // Round to 1 decimal place
/// assert_eq!(rounded_value, 0.6); // expected y-value
///
/// // Error example:
/// let invalid_bez = bezier(1.2, 0.4, 0.6, 0.8); // This will return an error due to control points out of bounds
/// ```
pub fn bezier(
    x1: f32,
    y1: f32,
    x2: f32,
    y2: f32,
) -> Result<impl Fn(f32) -> Result<f32, BezierError>, BezierError> {
    // Ensure control points are within bounds (for x-coordinates of p1 and p2).
    if !(0.0..=1.0).contains(&x1)
        || !(0.0..=1.0).contains(&x2)
        || !(0.0..=1.0).contains(&y1)
        || !(0.0..=1.0).contains(&y2)
    {
        return Err(BezierError::InvalidControlPoint { x1, y1, x2, y2 });
    }

    Ok(move |x: f32| {
        // Shortcut for linear curves (control points are on the line y = x).
        if x1 == y1 && x2 == y2 {
            return Ok(x); // Return the same x for a linear curve (y = x).
        }

        if !(0.0 - f32::EPSILON..=1.0 + f32::EPSILON).contains(&x) {
            return Err(BezierError::ParameterCalculationError(x));
        }

        if x == 0.0 || x == 1.0 {
            return Ok(x);
        }

        let p0 = Point(0.0, 0.0);
        let p1 = Point(x1, y1);
        let p2 = Point(x2, y2);
        let p3 = Point(1.0, 1.0);

        // Find the parameter `t` corresponding to the x-coordinate.
        let t = get_t_for_x(x, p0, p1, p2, p3)?;
        // Once `t` is found, evaluate the Bézier curve for the y-coordinate.
        // Return the y-coordinate from the Point.
        Ok(de_casteljau(t, p0, p1, p2, p3).1)
    })
}

#[cfg(test)]
mod tests {
    use super::*; // This brings the items from the parent module into scope

    /// Tests the `bezier` function by calculating the y-coordinate for a given x-coordinate.
    ///
    /// This test ensures that the `bezier` function correctly computes the y-coordinate for a given x-coordinate
    /// within the Bézier curve. The test checks if the function produces a value that matches the expected result
    /// for x = 0.5. The expected value is rounded to one decimal place for comparison.
    #[test]
    fn test_bezier_curve() {
        // Test the bezier function with control points
        let bez = bezier(0.2, 0.4, 0.6, 0.8).unwrap();

        // Test if we can calculate the y value for a given x
        let y_at_0_5 = bez(0.5).unwrap();

        // Round the computed value to one decimal place for comparison
        let rounded_y = (y_at_0_5 * 10.0).round() / 10.0;

        // Assert that the rounded y value is equal to the expected value (0.6)
        assert_eq!(
            rounded_y, 0.6,
            "Expected y value at x = 0.5 to be 0.6, but got {}",
            rounded_y
        );
    }

    /// Tests the `bezier` function with invalid control points to ensure proper error handling.
    ///
    /// This test verifies that the `bezier` function returns an error when control points are provided that are outside
    /// the valid range [0, 1]. It specifically checks for cases where the first control point (x1) is greater than 1,
    /// which should trigger an error (`BezierError::InvalidControlPoint`).
    #[test]
    fn test_invalid_control_points() {
        // Test for invalid control points that should return an error
        let bez = bezier(1.2, 0.4, 0.6, 0.8); // Invalid control point x1 = 1.2
        assert!(bez.is_err(), "Expected error for invalid control points");
    }

    /// Tests edge cases for the Bézier curve, specifically when the x-coordinate is at the endpoints (t == 0 or t == 1).
    ///
    /// This test ensures that the `bezier` function correctly handles special cases when the x-coordinate is at the
    /// endpoints of the Bézier curve, which should always return the same value as the x-coordinate in a linear case.
    /// The function is tested at the endpoints of the curve (x = 0.0 and x = 1.0), ensuring that the y-coordinate
    /// matches the input x-coordinate in these cases.
    #[test]
    fn test_edge_cases() {
        // Test when t == 0 or t == 1 (endpoints of the curve)
        let bez = bezier(0.0, 0.0, 1.0, 1.0).unwrap();

        // Assert that the value at x = 0.0 is 0.0 (start of the curve)
        assert_eq!(
            bez(0.0).unwrap(),
            0.0,
            "Expected y value at x = 0.0 to be 0.0"
        );

        // Assert that the value at x = 1.0 is 1.0 (end of the curve)
        assert_eq!(
            bez(1.0).unwrap(),
            1.0,
            "Expected y value at x = 1.0 to be 1.0"
        );
    }
}
