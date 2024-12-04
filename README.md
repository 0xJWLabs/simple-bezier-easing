# simple-bezier-easing

A Rust library for calculating cubic Bézier curves using De Casteljau's algorithm, with error handling and binary subdivision support for finding curve parameters. This library is ideal for applications like animations, easing functions, and graphics rendering.

## Features

- **Cubic Bézier Curves**: Easily calculate Bézier curves with customizable control points.
- **De Casteljau's Algorithm**: Accurate and efficient curve evaluation.
- **Error Handling**: Handles invalid control points gracefully.
- **Binary Subdivision**: Finds parameters for given `x` values with high precision.

## Installation

To add this library to your project, include it in your `Cargo.toml` file:

```toml
[dependencies]
simple-bezier-easing = "0.1.0"
```

## Example Usage

```rs
use simple_bezier_easing::bezier;

fn main() {
    // Define a Bézier curve with control points
    let bez = bezier(0.2, 0.4, 0.6, 0.8).unwrap();

    // Calculate the y-coordinate for x = 0.5
    let y_at_0_5 = bez(0.5).unwrap();
    println!("y at x = 0.5: {}", y_at_0_5);
}
```

## Error Handling

The library returns an error for invalid control points, for example:

```rs
let invalid_bez = bezier(1.2, 0.4, 0.6, 0.8); // Invalid control point
assert!(invalid_bez.is_err(), "Expected error for invalid control points");
```

## License

This project is licensed under the MIT License. See the [LICENSE](./LICENSE) file for more details.
