import 'dart:math';
import 'package:dotted_line/dotted_line.dart';
import 'package:flutter/material.dart';
import 'package:titlebar_buttons/titlebar_buttons.dart';
import 'package:window_manager/window_manager.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  await windowManager.ensureInitialized();

  WindowOptions windowOptions = const WindowOptions(
    size: Size(500, 300),
    minimumSize: Size(500, 300),
    center: true,
    backgroundColor: Colors.transparent,
    skipTaskbar: false,
    titleBarStyle: TitleBarStyle.normal,
    windowButtonVisibility: true,
  );

  windowManager.waitUntilReadyToShow(windowOptions, () async {
    await windowManager.show();
    await windowManager.focus();
  });

  runApp(const LauncherVisApp());
}

class LauncherVisApp extends StatelessWidget {
  const LauncherVisApp({super.key});

  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Beast Note Launching Diagnostics',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
          brightness: Brightness.dark,
          colorSchemeSeed: Colors.cyan.shade400,
          useMaterial3: true),
      home: const MyHomePage(title: 'Beast Note Launching Diagnostics'),
    );
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key, required this.title});

  // This widget is the home page of your application. It is stateful, meaning
  // that it has a State object (defined below) that contains fields that affect
  // how it looks.

  // This class is the configuration for the state. It holds the values (in this
  // case the title) provided by the parent (in this case the App widget) and
  // used by the build method of the State. Fields in a Widget subclass are
  // always marked "final".

  final String title;

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  int _counter = 0;

  void _incrementCounter() {
    setState(() {
      // This call to setState tells the Flutter framework that something has
      // changed in this State, which causes it to rerun the build method below
      // so that the display can reflect the updated values. If we changed
      // _counter without calling setState(), then the build method would not be
      // called again, and so nothing would appear to happen.
      _counter++;
    });
  }

  @override
  Widget build(BuildContext context) {
    // This method is rerun every time setState is called, for instance as done
    // by the _incrementCounter method above.
    //
    // The Flutter framework has been optimized to make rerunning build methods
    // fast, so that you can just rebuild anything that needs updating rather
    // than having to individually change instances of widgets.

    return Scaffold(
        appBar: AppBar(
            backgroundColor: Theme.of(context).colorScheme.inversePrimary,
            title: Center(child: Text(widget.title, style: TextStyle(fontSize: 20))),
            // actions: const [
            //   Row(children: [
            //     Padding(
            //       padding: EdgeInsets.fromLTRB(0, 0, 5, 4),
            //       child: Text("Settings", style: TextStyle(fontSize: 15)),
            //     ),
            //     Icon(Icons.settings)
            //   ]),
            //   SizedBox(width: 20),
            // ]
			),
        body: Container(
          //   width: 500,
          //   height: 200,
          // color: Theme.of(context).scaffoldBackgroundColor,
          padding: const EdgeInsets.fromLTRB(20, 10, 20, 10),
          child: RobotVisualizationWidgit(),
        ));
  }
}

class RobotVisualizationWidgit extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Container(
        padding: const EdgeInsets.fromLTRB(0, 10, 0, 10),
        child: Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              Container(
                constraints: const BoxConstraints(minWidth: 50, maxWidth: 500),
                decoration: BoxDecoration(
                    borderRadius: BorderRadius.circular(5),
                    color: Theme.of(context).hoverColor),
                // color: Theme.of(context).hoverColor,
                padding: const EdgeInsets.all(20),
                child: AspectRatio(
                  aspectRatio: 1,
                  child: Expanded(
                    flex: 2,
                    child: Container(
                      child: Align(
                        child: Stack(
                          alignment: Alignment.center,
                          children: [
                            Transform.rotate(
                              alignment: const Alignment(0, 0),
                              angle: -10 * pi / 180,
                              child: DottedLine(
                                direction: Axis.horizontal,
                                dashColor: Colors.red.shade500,
                                // dashColor: Colors.lightGreen,
                                dashLength: 20,
                                dashGapLength: 10,
                                lineThickness: 3,
                              ),
                            ),
                            Transform.rotate(
                              alignment: const Alignment(-0.165, 0.275),
                              angle: -60 * pi / 180,
                              child: const Image(
                                  opacity: AlwaysStoppedAnimation(0.2),
                                  image: AssetImage(
                                      "assets/RobotRightLauncher.png"),
                                  height: 250),
                            ),
                            Transform.rotate(
                              alignment: const Alignment(-0.17, 0.275),
                              angle: -10 * pi / 180,
                              child: const Image(
                                  image: AssetImage(
                                      "assets/RobotRightLauncher.png"),
                                  height: 250),
                            ),
                            const Image(
                                // opacity: AlwaysStoppedAnimation(0.5),
                                image: AssetImage("assets/RobotRightBase.png"),
                                height: 250),
                          ],
                        ),
                      ),
                    ),
                  ),
                ),
              ),
              //   const Expanded(
              // 	flex: 1,
              //     child: DottedLine(
              //       direction: Axis.horizontal,
              //       dashColor: Colors.blue,
              //       dashLength: 10,
              //       dashGapLength: 15,
              //       lineThickness: 50,
              //     ),
              //   ),
              //   const Expanded(
              //     child: Align(
              //         alignment: Alignment.center,
              //         child: Text("Distance from Speaker:")),
              //   )
              //   const Spacer(),
              const SizedBox(width: 20),
              Expanded(
                  flex: 2,
                  child: Container(
                      alignment: Alignment.topLeft,
                      padding: const EdgeInsets.all(10),
                      decoration: BoxDecoration(
                          borderRadius: BorderRadius.circular(5),
                          color: Theme.of(context).hoverColor),
                      child: Align(child: Column(mainAxisAlignment: MainAxisAlignment.center, children: const [ Text("Not Connected to NetworkTables")])),
                      //   child: Column(
                      // 	crossAxisAlignment: CrossAxisAlignment.start,
                      //     children: [
                      //       Column(
                      //         crossAxisAlignment: CrossAxisAlignment.start,
                      //         children: [
                      //           const Text("ROTATION",
                      //               style: TextStyle(
                      //                   fontSize: 18, color: Colors.white54)),
                      //           Container(
                      //               margin: const EdgeInsets.fromLTRB(0, 5, 0, 10),
                      //               padding:
                      //                   const EdgeInsets.fromLTRB(10, 5, 10, 5),
                      //               decoration: BoxDecoration(
                      //                 color: Theme.of(context).focusColor,
                      //                 borderRadius: BorderRadius.circular(2),
                      //               ),
                      //               child: const Text("10 (Degrees)"))
                      //         ],
                      //       ),
                      //       Column(
                      //           crossAxisAlignment: CrossAxisAlignment.start,
                      //           children: [
                      //             const Text("NOTE LOADED",
                      //                 style: TextStyle(
                      //                     fontSize: 18, color: Colors.white54)),
                      //             Container(
                      //               margin: const EdgeInsets.fromLTRB(0, 5, 0, 5),
                      //               padding:
                      //                   const EdgeInsets.fromLTRB(2, 2, 2, 2),
                      //               decoration: BoxDecoration(
                      //                 color: Theme.of(context).focusColor,
                      //                 borderRadius: BorderRadius.circular(2),
                      //               ),
                      //               child: SizedBox(
                      //                 width: 100,
                      //                 height: 20,
                      //                 child: Container(color: Colors.green),
                      //               ),
                      //             )
                      //           ])
                      //     ],
                      //   )
                      ))
            ]));
  }
}
