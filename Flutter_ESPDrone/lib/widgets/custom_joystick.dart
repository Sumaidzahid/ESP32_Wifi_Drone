import 'package:flutter/material.dart';

class CustomJoystick extends StatefulWidget {
  final double size;
  final Color baseColor;
  final Color stickColor;
  final Function(double x, double y) listener;
  final bool stickyY; // If true, Y-axis won't snap back
  final Color? borderColor;
  final double borderWidth;
  final Color? shadowColor;
  final Color? stickShadowColor;
  final VoidCallback? onPanStart;
  final VoidCallback? onPanEnd;

  const CustomJoystick({
    super.key,
    this.size = 120,
    required this.baseColor,
    required this.stickColor,
    required this.listener,
    this.stickyY = false,
    this.borderColor,
    this.borderWidth = 0,
    this.shadowColor,
    this.stickShadowColor,
    this.onPanStart,
    this.onPanEnd,
  });

  @override
  State<CustomJoystick> createState() => _CustomJoystickState();
}

class _CustomJoystickState extends State<CustomJoystick> {
  Offset stickOffset = Offset.zero;

  @override
  Widget build(BuildContext context) {
    double radius = widget.size / 2;

    return GestureDetector(
      onPanStart: (_) {
        widget.onPanStart?.call();
      },
      onPanUpdate: (details) {
        setState(() {
          Offset newOffset = stickOffset + details.delta;

          // Keep inside joystick circle
          if (newOffset.distance > radius) {
            newOffset = Offset.fromDirection(newOffset.direction, radius);
          }

          stickOffset = Offset(
            newOffset.dx,
            widget.stickyY ? stickOffset.dy + details.delta.dy : newOffset.dy,
          );

          // For stickyY, clamp y to radius
          if (widget.stickyY && stickOffset.dy.abs() > radius) {
            stickOffset = Offset(stickOffset.dx, stickOffset.dy.sign * radius);
          }

          // Normalized values -1 -> 1
          double xNorm = stickOffset.dx / radius;
          double yNorm = stickOffset.dy / radius;

          widget.listener(xNorm.clamp(-1.0, 1.0), yNorm.clamp(-1.0, 1.0));
        });
      },
      onPanEnd: (_) {
        setState(() {
          // X-axis springs back, Y-axis stays if sticky
          stickOffset = Offset(0, widget.stickyY ? stickOffset.dy : 0);
        });
        widget.onPanEnd?.call();
      },
      child: Container(
        width: widget.size,
        height: widget.size,
        decoration: BoxDecoration(
          color: widget.baseColor,
          shape: BoxShape.circle,
          border: widget.borderColor != null ? Border.all(color: widget.borderColor!, width: widget.borderWidth) : null,
          boxShadow: widget.shadowColor != null ? [
            BoxShadow(color: widget.shadowColor!.withOpacity(0.3), blurRadius: 8),
          ] : null,
        ),
        child: Center(
          child: Transform.translate(
            offset: stickOffset,
            child: Container(
              width: widget.size * 0.4,
              height: widget.size * 0.4,
              decoration: BoxDecoration(
                color: widget.stickColor,
                shape: BoxShape.circle,
                boxShadow: widget.stickShadowColor != null ? [
                  BoxShadow(
                    color: widget.stickShadowColor!.withOpacity(0.3),
                    blurRadius: 4,
                  ),
                ] : null,
              ),
            ),
          ),
        ),
      ),
    );
  }
}
