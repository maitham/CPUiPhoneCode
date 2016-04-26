//
//  Use this file to import your target's public headers that you would like to expose to Swift.
//

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>


@interface ImageProcessing: NSObject
- (id)init;
- (UIImage *)recognizeCars:(UIImage *)image;
- (UIImage *)carryOutHough:(UIImage *)image;
- (UIImage *)carryOutInversePerspectiveMap:(UIImage *)image;
- (UIImage *)determineVanishingPoints: (UIImage *)image;
- (UIImage *)detectLanesEffeciently:(UIImage *)image :(UIImage *)image2;
- (UIImage *)detectLanesGetOpticFlow:(UIImage *)image;

@end