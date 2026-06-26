import 'package:flutter/material.dart';
import 'dart:async';
import 'dart:io';
import 'package:flutter/services.dart';
import 'widgets/custom_joystick.dart';

void main() {
  WidgetsFlutterBinding.ensureInitialized();

  SystemChrome.setPreferredOrientations([
    DeviceOrientation.landscapeLeft,
    DeviceOrientation.landscapeRight,
  ]);

  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      home: const ControllerPage(),
    );
  }
}

class ControllerPage extends StatefulWidget {
  const ControllerPage({super.key});

  @override
  State<ControllerPage> createState() => _ControllerPageState();
}

class _ControllerPageState extends State<ControllerPage> {
  final TextEditingController ipController = TextEditingController(text: "192.168.4.1");

  // Control values
  double throttle = 0;
  double yaw = 0;
  double pitch = 0;
  double roll = 0;

  // Safety flag
  bool isArmed = false;

  Timer? _sendTimer;

  RawDatagramSocket? _socket;
  InternetAddress? _espAddress;
  final int _espPort = 4210;

  void sendControl() {
    if (!isArmed || _socket == null || _espAddress == null) return;

    final dataString =
        "${throttle.toStringAsFixed(2)},"
        "${yaw.toStringAsFixed(2)},"
        "${pitch.toStringAsFixed(2)},"
        "${roll.toStringAsFixed(2)}";

    final data = dataString.codeUnits;

    _socket!.send(data, _espAddress!, _espPort);
    

  }

  void sendZeroPacket() {
    if (_socket == null || _espAddress == null) return;

    final zeroData = "0,0,0,0".codeUnits;
    _socket!.send(zeroData, _espAddress!, _espPort);
  }

  void sendArmStatus(bool armed) {
    if (_socket == null || _espAddress == null) return;

    final dataString = "ARMED:${armed ? 'true' : 'false'}";
    final data = dataString.codeUnits;
    _socket!.send(data, _espAddress!, _espPort);
  }

  Future<void> startUDP() async {
    _espAddress = InternetAddress(ipController.text);
    _socket = await RawDatagramSocket.bind(
      InternetAddress.anyIPv4,
      0,
    );
  }

  @override
  void dispose() {
    _sendTimer?.cancel();
    _socket?.close();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    // Color palette from user request
    const Color backgroundColor = Color(0xFF0F1115);

    // Local background image path (Windows)
    const String bgImagePath = r'C:\Users\sumai\OneDrive\Pictures\Screenshots\a neon light transpa.png';
    // Bundled asset path (preferred when available)
    const String assetBgPath = 'assets/images/a neon light transpa.png';

    Color leftBase = Color(0xFF1F3D2B).withValues(alpha: 0.6);
    const Color leftStick = Color(0xFF2ECC71);
    Color leftBorder = Color(0xFF27AE60).withValues(alpha: 0.8);

    Color rightBase = Color(0xFF1E2A3A).withValues(alpha: 0.6);
    const Color rightStick = Color(0xFF3498DB);
    Color rightBorder = Color(0xFF2980B9).withValues(alpha: 0.8);

    const Color armButtonColor = Color(0xFFE74C3C);
    const Color calibButtonColor = Color(0xFFF1C40F);

    const Color primaryText = Color(0xFFECF0F1);
    const Color hintText = Color(0xFF7F8C8D);

    const Color inputBorder = Color(0xFF34495E);
    const Color inputText = Color(0xFF2C3E50);

    return Scaffold(
      backgroundColor: backgroundColor,
      body: Stack(
        children: [
          // Background image from local path (falls back to solid color if missing)
          Positioned.fill(
            child: Builder(builder: (context) {
              // Prefer asset image (works on all platforms once bundled).
              try {
                return Image.asset(assetBgPath, fit: BoxFit.cover);
              } catch (_) {}

              // Fallback to local file path for desktop development.
              try {
                final file = File(bgImagePath);
                if (file.existsSync()) {
                  return Image.file(file, fit: BoxFit.cover);
                }
              } catch (_) {}

              return Container(color: backgroundColor);
            }),
          ),
          // a slight overlay to improve contrast
          Positioned.fill(child: Container(color: Colors.black.withValues(alpha: 0.09))),
          SafeArea(
            child: Column(
              children: [

            // 🔹 TOP: Label + TextBox
                  Center(
              child: Padding(
                padding: const EdgeInsets.all(8.0),
                child: Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  crossAxisAlignment: CrossAxisAlignment.center,
                  children: [
                        const Text(
                          "Drone IP Address",
                          style: TextStyle(color: primaryText),
                        ),
                        const SizedBox(width: 10),
                        SizedBox(
                          width: MediaQuery.of(context).size.width * 0.2,
                          child: TextField(
                            controller: ipController,
                            style: const TextStyle(color: inputText),
                            decoration: InputDecoration(
                              hintText: "192.168.4.1",
                              hintStyle: const TextStyle(color: hintText),
                              enabledBorder: OutlineInputBorder(
                                borderSide: const BorderSide(color: inputBorder),
                                borderRadius: BorderRadius.circular(6),
                              ),
                              focusedBorder: OutlineInputBorder(
                                borderSide: const BorderSide(color: inputBorder, width: 2),
                                borderRadius: BorderRadius.circular(6),
                              ),
                            ),
                          ),
                        ),
                  ],
                ),
              ),
            ),

            // 🔹 MAIN AREA
            Expanded(
              child: Column(
                children: [
                  Expanded(
                    child: Row(
                      mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                      children: [

                        // LEFT JOYSTICK
                        CustomJoystick(
                          size: 200,
                          baseColor: leftBase,
                          stickColor: leftStick,
                          borderColor: leftBorder,
                          borderWidth: 4,
                          shadowColor: Color(0xFF27AE60).withValues(alpha: 0.5),
                          stickShadowColor: leftBorder.withValues(alpha: 0.6),
                          stickyY: true, // Throttle stays
                          listener: (x, y) {
                            if (!isArmed) return;
                            setState(() {
                              throttle = ((-y + 1) / 2).clamp(0.0, 1.0);
                              yaw = x.clamp(-1.0, 1.0);
                            });
                          },
                          onPanEnd: () {
                            setState(() {
                              yaw = 0; // X-axis springs back
                            });
                          },
                        ),


                        // CENTER BUTTONS
                        Column(
                          mainAxisAlignment: MainAxisAlignment.center,
                          children: [
                            ElevatedButton(
                              onPressed: () async {
                                await startUDP();
                                setState(() {
                                  isArmed = true;
                                  _sendTimer = Timer.periodic(const Duration(milliseconds: 10), (_) => sendControl());
                                });
                                sendArmStatus(true);
                              },
                              style: ElevatedButton.styleFrom(backgroundColor: armButtonColor),
                              child: const Text("ARM", style: TextStyle(color: Color(0xFFECF0F1))),
                            ),
                            const SizedBox(height: 10),
                            ElevatedButton(
                              onPressed: () {
                                setState(() {
                                  isArmed = false;
                                  throttle = 0;
                                  yaw = 0;
                                  pitch = 0;
                                  roll = 0;
                                });
                                sendArmStatus(false);
                                _sendTimer?.cancel();
                                sendZeroPacket();
                                _socket?.close();
                                _socket = null;
                              },
                              style: ElevatedButton.styleFrom(backgroundColor: Colors.grey[800]),
                              child: const Text("DISARM", style: TextStyle(color: primaryText)),
                            ),
                            const SizedBox(height: 10),
                            ElevatedButton(
                              onPressed: () {},
                              style: ElevatedButton.styleFrom(backgroundColor: calibButtonColor),
                              child: const Text("CALIB", style: TextStyle(color: Color(0xFF2C3E50))),
                            ),
                          ],
                        ),

                        // RIGHT JOYSTICK
                        CustomJoystick(
                          size: 200,
                          baseColor: rightBase,
                          stickColor: rightStick,
                          borderColor: rightBorder,
                          borderWidth: 4,
                          shadowColor: Color(0xFF2980B9).withValues(alpha: 0.5),
                          stickShadowColor: rightBorder.withValues(alpha: 0.6),
                          stickyY: false, // Both axes spring back
                          listener: (x, y) {
                            if (!isArmed) return;
                            setState(() {
                              roll = x.clamp(-1.0, 1.0);
                              pitch = (-y).clamp(-1.0, 1.0);
                            });
                          },
                          onPanEnd: () {
                            setState(() {
                              roll = 0;
                              pitch = 0;
                            });
                          },
                        ),


                      ],
                    ),
                  ),

                  // OUTPUT DISPLAY
                  Container(
                    padding: const EdgeInsets.all(8.0),
                    color: Colors.black.withValues(alpha: 0.5),
                    child: Row(
                      mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                      children: [
                        Text("Throttle: ${throttle.toStringAsFixed(2)}", style: const TextStyle(color: primaryText)),
                        Text("Yaw: ${yaw.toStringAsFixed(2)}", style: const TextStyle(color: primaryText)),
                        Text("Pitch: ${pitch.toStringAsFixed(2)}", style: const TextStyle(color: primaryText)),
                        Text("Roll: ${roll.toStringAsFixed(2)}", style: const TextStyle(color: primaryText)),
                        Text("Armed: ${isArmed ? 'YES' : 'NO'}", style: TextStyle(color: isArmed ? Colors.green : Colors.red)),
                      ],
                    ),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    ],
  ),
    );
  }
}
