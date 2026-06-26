// This is a basic Flutter widget test.
//
// To perform an interaction with a widget in your test, use the WidgetTester
// utility in the flutter_test package. For example, you can send tap and scroll
// gestures. You can also use WidgetTester to find child widgets in the widget
// tree, read text, and verify that the values of widget properties are correct.

import 'package:flutter_test/flutter_test.dart';

import 'package:espdrone/main.dart';

void main() {
  testWidgets('Controller UI smoke test', (WidgetTester tester) async {
    // Build our app and trigger a frame.
    await tester.pumpWidget(const MyApp());

    expect(find.text('Drone IP Address'), findsOneWidget);
    expect(find.text('ARM'), findsOneWidget);
    expect(find.text('DISARM'), findsOneWidget);
    expect(find.text('CALIB'), findsOneWidget);
    expect(find.text('Armed: NO'), findsOneWidget);
  });
}
