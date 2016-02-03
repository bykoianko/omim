#import "Common.h"
#import "MapsAppDelegate.h"
#import "MapViewController.h"
#import "ViewController.h"
#import "3party/Alohalytics/src/alohalytics_objc.h"

@implementation ViewController

- (BOOL)prefersStatusBarHidden
{
  return NO;
}

- (void)refresh
{
  [self.navigationController.navigationBar refresh];
  [self.view refresh];
  if (![self isKindOfClass:[MapViewController class]])
    [[MapsAppDelegate theApp].mapViewController refresh];
}

- (void)viewDidLoad
{
  [super viewDidLoad];
  [self.navigationController.navigationBar setTranslucent:NO];
}

- (void)viewWillAppear:(BOOL)animated
{
  [Alohalytics logEvent:@"$viewWillAppear" withValue:NSStringFromClass([self class])];
  [super viewWillAppear:animated];
}

- (void)viewWillDisappear:(BOOL)animated
{
  [Alohalytics logEvent:@"$viewWillDisappear" withValue:NSStringFromClass([self class])];
  [super viewWillDisappear:animated];
}

- (void)showAlert:(NSString *)alert withButtonTitle:(NSString *)buttonTitle
{
  if (isIOS7)
  {
    UIAlertView * alertView = [[UIAlertView alloc] initWithTitle:alert
                                                         message:nil
                                                        delegate:nil
                                               cancelButtonTitle:buttonTitle
                                               otherButtonTitles:nil];
    [alertView show];
  }
  else
  {
    UIAlertController * alertController =
        [UIAlertController alertControllerWithTitle:alert
                                            message:nil
                                     preferredStyle:UIAlertControllerStyleAlert];
    UIAlertAction * okAction =
        [UIAlertAction actionWithTitle:buttonTitle style:UIAlertActionStyleCancel handler:nil];
    [alertController addAction:okAction];
    [self presentViewController:alertController animated:YES completion:nil];
  }
}

#pragma mark - Properties

- (BOOL)hasNavigationBar
{
  return YES;
}

@end