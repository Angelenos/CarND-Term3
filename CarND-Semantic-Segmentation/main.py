#!/usr/bin/env python3
import os.path
import re
import tensorflow as tf
import helper
import warnings
from glob import glob
from distutils.version import LooseVersion
from sklearn.model_selection import train_test_split
import project_tests as tests


# Check TensorFlow Version
assert LooseVersion(tf.__version__) >= LooseVersion('1.0'), \
    'Please use TensorFlow version 1.0 or newer. You are using {}'.format(tf.__version__)
print('TensorFlow Version: {}'.format(tf.__version__))

# Check for a GPU
if not tf.test.gpu_device_name():
    warnings.warn('No GPU found. Please use a GPU to train your neural network.')
else:
    print('Default GPU Device: {}'.format(tf.test.gpu_device_name()))


def load_vgg(sess, vgg_path):
    """
    Load Pretrained VGG Model into TensorFlow.
    :param sess: TensorFlow Session
    :param vgg_path: Path to vgg folder, containing "variables/" and "saved_model.pb"
    :return: Tuple of Tensors from VGG model (image_input, keep_prob, layer3_out, layer4_out, layer7_out)
    """
    # TODO: Implement function
    #   Use tf.saved_model.loader.load to load the model and weights
    vgg_tag = 'vgg16'
    vgg_input_tensor_name = 'image_input:0'
    vgg_keep_prob_tensor_name = 'keep_prob:0'
    vgg_layer3_out_tensor_name = 'layer3_out:0'
    vgg_layer4_out_tensor_name = 'layer4_out:0'
    vgg_layer7_out_tensor_name = 'layer7_out:0'

    tf.saved_model.loader.load(sess, [vgg_tag], vgg_path)

    vgg_input_tensor = tf.get_default_graph().get_tensor_by_name(vgg_input_tensor_name)
    vgg_keep_prob_tensor = tf.get_default_graph().get_tensor_by_name(vgg_keep_prob_tensor_name)
    vgg_layer3_out_tensor = tf.get_default_graph().get_tensor_by_name(vgg_layer3_out_tensor_name)
    vgg_layer4_out_tensor = tf.get_default_graph().get_tensor_by_name(vgg_layer4_out_tensor_name)
    vgg_layer7_out_tensor = tf.get_default_graph().get_tensor_by_name(vgg_layer7_out_tensor_name)
    
    return vgg_input_tensor, vgg_keep_prob_tensor, vgg_layer3_out_tensor, vgg_layer4_out_tensor, vgg_layer7_out_tensor


tests.test_load_vgg(load_vgg, tf)


def layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes):
    """
    Create the layers for a fully convolutional network.  Build skip-layers using the vgg layers.
    :param vgg_layer3_out: TF Tensor for VGG Layer 3 output
    :param vgg_layer4_out: TF Tensor for VGG Layer 4 output
    :param vgg_layer7_out: TF Tensor for VGG Layer 7 output
    :param num_classes: Number of classes to classify
    :return: The Tensor for the last layer of output
    """
    # TODO: Implement function
    # 1x1 Fully convolution layer
    conv_1x1 = tf.layers.conv2d(vgg_layer7_out, num_classes, 1, strides=(1, 1), padding='same',
                                kernel_initializer=tf.random_normal_initializer(stddev=0.01),
                                kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-4))

    # Decode the vgg_layer4
    output = tf.layers.conv2d_transpose(conv_1x1, num_classes, 4, strides=(2, 2), padding='same',
                                        kernel_initializer=tf.random_normal_initializer(stddev=0.01),
                                        kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-4))

    # Resize the vgg_layer4 to match the num_class output and apply the weights of 0.01
    vgg_layer4_out_resize = tf.layers.conv2d(vgg_layer4_out, num_classes, 1, padding='same',
                                             kernel_initializer=tf.random_normal_initializer(stddev=0.01),
                                             kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-4))
    vgg_layer4_out_resize = tf.multiply(vgg_layer4_out_resize, 0.01)

    # Apply skip connection to the output layer
    output = tf.add(output, vgg_layer4_out_resize)

    # Decode the vgg_layer3
    output = tf.layers.conv2d_transpose(output, num_classes, 4, strides=(2, 2), padding='same',
                                        kernel_initializer=tf.random_normal_initializer(stddev=0.01),
                                        kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-4))

    # Resize the vgg_layer3 to match the num_class output and apply the weights of 0.0001 
    vgg_layer3_out_resize = tf.layers.conv2d(vgg_layer3_out, num_classes, 1, padding='same',
                                             kernel_initializer=tf.random_normal_initializer(stddev=0.01),
                                             kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-4))
    vgg_layer3_out_resize = tf.multiply(vgg_layer3_out_resize, 0.0001)

    # Apply skip connection to the output layer
    output = tf.add(output, vgg_layer3_out_resize)

    # Last step of decoding
    output = tf.layers.conv2d_transpose(output, num_classes, 16, strides=(8, 8), padding='same',
                                        kernel_initializer=tf.random_normal_initializer(stddev=0.01),
                                        kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-4))

    return output


tests.test_layers(layers)


def optimize(nn_last_layer, correct_label, learning_rate, num_classes):
    """
    Build the TensorFLow loss and optimizer operations.
    :param nn_last_layer: TF Tensor of the last layer in the neural network
    :param correct_label: TF Placeholder for the correct label image
    :param learning_rate: TF Placeholder for the learning rate
    :param num_classes: Number of classes to classify
    :return: Tuple of (logits, train_op, cross_entropy_loss, iou_prediction, iou_update)
    """
    # TODO: Implement function
    logits = nn_last_layer
    cross_entropy = tf.nn.softmax_cross_entropy_with_logits(labels=correct_label, logits=logits)
    loss_operation = tf.reduce_mean(cross_entropy)
    optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate) # Use Adam optimizater with learning_rate specified
    training_operation = optimizer.minimize(loss_operation)

    # TODO: Implement IoU Accuracy Evaludation
    iou_prediction, iou_update = tf.metrics.mean_iou(tf.argmax(correct_label, -1), tf.argmax(logits, -1), num_classes)

    return logits, training_operation, loss_operation, iou_prediction, iou_update


tests.test_optimize(optimize)


def train_nn(sess, epochs, batch_size, get_batches_fn_train, get_batches_fn_val, train_op, cross_entropy_loss,
             iou_prediction, iou_update, input_image, correct_label, keep_prob, learning_rate):
    """
    Train neural network and print out the loss during training.
    :param sess: TF Session
    :param epochs: Number of epochs
    :param batch_size: Batch size
    :param get_batches_fn_train: Function to get batches of training data.  Call using get_batches_fn(batch_size)
    :param get_batches_fn_val: Function to get batches of validation data.  Call using get_batches_fn(batch_size)
    :param train_op: TF Operation to train the neural network
    :param cross_entropy_loss: TF Tensor for the amount of loss
    :param iou_prediction: TF Tensor for the validation accuracy
    :param iou_update: TF Tensor Operation to generate the ioe result
    :param input_image: TF Placeholder for input images
    :param correct_label: TF Placeholder for label images
    :param keep_prob: TF Placeholder for dropout keep probability
    :param learning_rate: TF Placeholder for learning rate
    """
    # TODO: Implement function
    sess.run(tf.global_variables_initializer())
    sess.run(tf.local_variables_initializer())

    print("Start training...")
    for epoch in range(epochs):
        # Training under given batch_size
        for image, label in get_batches_fn_train(batch_size):
            feed = {input_image: image, correct_label: label, learning_rate: 0.0001, keep_prob: 0.8}
            sess.run(train_op, feed_dict=feed)

        # Evaluting the model behavior with the validation set specified
        num_val_sample = 0
        accuracy_tot = 0
        for image, label in get_batches_fn_val(batch_size):
            feed = {input_image: image, correct_label: label, keep_prob: 1.0}
            sess.run(iou_update, feed_dict=feed)  # Pre-requisite session run to calculate IoU value
            accuracy = sess.run(iou_prediction)
            num_val_sample += len(image)
            accuracy_tot += (accuracy * len(image))

        accuracy_tot /= num_val_sample # Average over all batches to get the mean accuracy under this epoch
        print("Epoch {}/{}: Validation Accuracy = {:.3f}".format(epoch + 1, epochs, accuracy_tot))

    pass


tests.test_train_nn(train_nn)


def run():
    num_classes = 2
    epochs = 20
    batch_size = 16
    image_shape = (160, 576)
    data_dir = './data'
    runs_dir = './runs'
    tests.test_for_kitti_dataset(data_dir)

    # Download pre-trained vgg model
    helper.maybe_download_pretrained_vgg(data_dir)

    # OPTIONAL: Train and Inference on the cityscapes dataset instead of the Kitti dataset.
    # You'll need a GPU with at least 10 teraFLOPS to train on.
    #  https://www.cityscapes-dataset.com/

    # Split the training data into training and validation set:
    image_paths = glob(os.path.join(data_dir, 'data_road/training/image_2/', '*.png'))
    label_paths = {
        re.sub(r'_(lane|road)_', '_', os.path.basename(path)): path
        for path in glob(os.path.join(data_dir, 'data_road/training/gt_image_2', '*_road_*.png'))}
    train_x, validation_x = train_test_split(image_paths, test_size=0.2)

    with tf.Session() as sess:
        # Path to vgg model
        vgg_path = os.path.join(data_dir, 'vgg')
        # Create function to get batches
        get_batches_fn_train = helper.gen_batch_function(train_x, label_paths, image_shape)
        get_batches_fn_val = helper.gen_batch_function(validation_x, label_paths, image_shape)

        # OPTIONAL: Augment Images for better results
        #  https://datascience.stackexchange.com/questions/5224/how-to-prepare-augment-images-for-neural-network

        # TODO: Build NN using load_vgg, layers, and optimize function
        correct_label = tf.placeholder(tf.int32, shape=(None, None, None, num_classes))
        learning_rate = tf.placeholder(tf.float32)

        input_image, keep_prob, layer3_out, layer4_out, layer7_out = load_vgg(sess, vgg_path)
        last_layer = layers(layer3_out, layer4_out, layer7_out, num_classes)
        logits, train_op, cross_entropy_loss, iou_prediction, iou_update = optimize(last_layer, correct_label,
                                                                                    learning_rate, num_classes)

        # TODO: Train NN using the train_nn function
        train_nn(sess, epochs, batch_size, get_batches_fn_train, get_batches_fn_val, train_op, cross_entropy_loss,
                 iou_prediction, iou_update, input_image, correct_label, keep_prob, learning_rate)

        # TODO: Save inference data using helper.save_inference_samples
        out_dir = helper.save_inference_samples(runs_dir, data_dir, sess, image_shape, logits, keep_prob, input_image)

        # Model saver
        saver = tf.train.Saver()
        saver.save(sess, out_dir + "/FCN")

        # OPTIONAL: Apply the trained model to a video


if __name__ == '__main__':
    run()
